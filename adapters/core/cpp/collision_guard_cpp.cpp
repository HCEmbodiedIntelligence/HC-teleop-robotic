#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/mesh_loader/loader.h>

#include <urdf_parser/urdf_parser.h>

#include <btBulletCollisionCommon.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace {

constexpr btScalar kUrdfCollisionMargin = btScalar(0.001);

struct CollisionHit {
  std::string link_a;
  std::string link_b;
  double distance;
  double clearance;

  double deficit() const { return clearance - distance; }
};

struct MinDistanceResult final : public btDiscreteCollisionDetectorInterface::Result {
  bool found = false;
  btScalar min_distance = btScalar(BT_LARGE_FLOAT);

  void setShapeIdentifiersA(int, int) override {}
  void setShapeIdentifiersB(int, int) override {}

  void addContactPoint(const btVector3&, const btVector3&, btScalar depth) override {
    if (!found || depth < min_distance) {
      found = true;
      min_distance = depth;
    }
  }
};

struct WorldAabb {
  btVector3 lo;
  btVector3 hi;
};

inline double aabb_dist_sq(const WorldAabb& a, const WorldAabb& b) {
  double sq = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    const double g1 = static_cast<double>(b.lo[axis]) - static_cast<double>(a.hi[axis]);
    const double g2 = static_cast<double>(a.lo[axis]) - static_cast<double>(b.hi[axis]);
    const double gap = std::max({g1, g2, 0.0});
    sq += gap * gap;
  }
  return sq;
}

inline btTransform se3_to_bt(const pinocchio::SE3& M) {
  const auto& R = M.rotation();
  const auto& t = M.translation();
  btMatrix3x3 basis(
      static_cast<btScalar>(R(0, 0)), static_cast<btScalar>(R(0, 1)), static_cast<btScalar>(R(0, 2)),
      static_cast<btScalar>(R(1, 0)), static_cast<btScalar>(R(1, 1)), static_cast<btScalar>(R(1, 2)),
      static_cast<btScalar>(R(2, 0)), static_cast<btScalar>(R(2, 1)), static_cast<btScalar>(R(2, 2)));
  btVector3 origin(static_cast<btScalar>(t[0]),
                   static_cast<btScalar>(t[1]),
                   static_cast<btScalar>(t[2]));
  return btTransform(basis, origin);
}

inline pinocchio::SE3 urdf_pose_to_se3(const ::urdf::Pose& p) {
  Eigen::Quaterniond q(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
  if (q.squaredNorm() > 1e-18) {
    q.normalize();
  } else {
    q = Eigen::Quaterniond::Identity();
  }
  return pinocchio::SE3(q.toRotationMatrix(),
                        Eigen::Vector3d(p.position.x, p.position.y, p.position.z));
}

inline double calibrated_clearance_cpp(double requested,
                                       std::optional<double> home_distance,
                                       double tolerance) {
  if (!home_distance.has_value() || *home_distance >= requested) {
    return requested;
  }
  if (*home_distance <= 0.0) {
    return 0.0;
  }
  return std::max(0.0, std::min(requested, *home_distance - tolerance));
}

struct Float3Key {
  uint32_t x, y, z;
  bool operator==(const Float3Key& o) const noexcept {
    return x == o.x && y == o.y && z == o.z;
  }
};

struct Float3KeyHash {
  std::size_t operator()(const Float3Key& k) const noexcept {
    std::size_t h = std::hash<uint32_t>{}(k.x);
    h ^= std::hash<uint32_t>{}(k.y) + 0x9e3779b9U + (h << 6) + (h >> 2);
    h ^= std::hash<uint32_t>{}(k.z) + 0x9e3779b9U + (h << 6) + (h >> 2);
    return h;
  }
};

// Fast binary STL reader returning deduplicated scaled vertices.
// Returns false if the file is not a valid binary STL (e.g. ASCII STL or DAE/OBJ).
bool try_read_binary_stl_vertices(const std::filesystem::path& path,
                                  double sx, double sy, double sz,
                                  std::vector<btVector3>& out_pts) {
  out_pts.clear();
  std::ifstream ifs(path, std::ios::binary | std::ios::ate);
  if (!ifs) return false;
  const std::streamsize fsize = ifs.tellg();
  if (fsize < 84) return false;
  ifs.seekg(0, std::ios::beg);
  std::vector<uint8_t> buf(static_cast<std::size_t>(fsize));
  if (!ifs.read(reinterpret_cast<char*>(buf.data()), fsize)) return false;

  uint32_t num_tris = 0;
  std::memcpy(&num_tris, buf.data() + 80, 4);
  const std::size_t expected = 84ULL + static_cast<std::size_t>(num_tris) * 50ULL;
  if (expected != buf.size() || num_tris == 0) {
    return false;
  }

  std::unordered_set<Float3Key, Float3KeyHash> seen;
  seen.reserve(std::min<std::size_t>(static_cast<std::size_t>(num_tris) * 2, 131072));
  out_pts.reserve(std::min<std::size_t>(static_cast<std::size_t>(num_tris), 65536));

  const uint8_t* ptr = buf.data() + 84;
  for (uint32_t t = 0; t < num_tris; ++t, ptr += 50) {
    // Each facet: 12B normal, then 3 vertices * 12B, then 2B attr.
    for (int v = 0; v < 3; ++v) {
      const uint8_t* vp = ptr + 12 + v * 12;
      Float3Key key;
      std::memcpy(&key.x, vp, 4);
      std::memcpy(&key.y, vp + 4, 4);
      std::memcpy(&key.z, vp + 8, 4);
      if (seen.insert(key).second) {
        float fx, fy, fz;
        std::memcpy(&fx, &key.x, 4);
        std::memcpy(&fy, &key.y, 4);
        std::memcpy(&fz, &key.z, 4);
        out_pts.emplace_back(static_cast<btScalar>(fx * sx),
                             static_cast<btScalar>(fy * sy),
                             static_cast<btScalar>(fz * sz));
      }
    }
  }
  return !out_pts.empty();
}

std::filesystem::path resolve_mesh_path(
    const std::string& filename,
    const std::filesystem::path& urdf_dir,
    const std::vector<std::filesystem::path>& pkg_dirs) {
  namespace fs = std::filesystem;
  if (filename.rfind("package://", 0) == 0) {
    const std::string rest = filename.substr(10);
    const std::size_t slash = rest.find('/');
    const std::string pkg_rel = (slash == std::string::npos) ? "" : rest.substr(slash + 1);
    for (const auto& dir : pkg_dirs) {
      const fs::path cand1 = dir / rest;
      if (fs::is_regular_file(cand1)) return fs::weakly_canonical(cand1);
      if (!pkg_rel.empty()) {
        const fs::path cand2 = dir / pkg_rel;
        if (fs::is_regular_file(cand2)) return fs::weakly_canonical(cand2);
      }
    }
  } else if (filename.rfind("file://", 0) == 0) {
    const fs::path cand(filename.substr(7));
    if (fs::is_regular_file(cand)) return fs::weakly_canonical(cand);
  } else {
    const fs::path raw(filename);
    if (raw.is_absolute() && fs::is_regular_file(raw)) {
      return fs::weakly_canonical(raw);
    }
    const fs::path rel = urdf_dir / raw;
    if (fs::is_regular_file(rel)) return fs::weakly_canonical(rel);
  }
  return {};
}

// Process-wide cache of optimized convex hull vertices keyed by (mesh_path, scale).
std::shared_ptr<const std::vector<btVector3>> load_cached_convex_hull_vertices(
    const std::filesystem::path& resolved_path,
    double sx, double sy, double sz) {
  static std::mutex cache_mu;
  static std::unordered_map<std::string, std::shared_ptr<const std::vector<btVector3>>> cache;

  const std::string key = resolved_path.string() + "|" +
                          std::to_string(sx) + "|" +
                          std::to_string(sy) + "|" +
                          std::to_string(sz);
  {
    std::lock_guard<std::mutex> lk(cache_mu);
    auto it = cache.find(key);
    if (it != cache.end()) {
      return it->second;
    }
  }

  std::vector<btVector3> pts;
  const std::string ext = resolved_path.extension().string();
  const bool is_stl = (ext == ".stl" || ext == ".STL");
  bool loaded = false;
  if (is_stl) {
    loaded = try_read_binary_stl_vertices(resolved_path, sx, sy, sz, pts);
  }
  if (!loaded) {
    hpp::fcl::MeshLoader loader(hpp::fcl::BV_AABB);
    const hpp::fcl::Vec3f scale_v(sx, sy, sz);
    auto bvh = loader.load(resolved_path.string(), scale_v);
    if (bvh && bvh->num_vertices > 0 && bvh->vertices != nullptr) {
      pts.reserve(static_cast<std::size_t>(bvh->num_vertices));
      for (unsigned int i = 0; i < bvh->num_vertices; ++i) {
        const auto& v = bvh->vertices[i];
        pts.emplace_back(static_cast<btScalar>(v[0]),
                         static_cast<btScalar>(v[1]),
                         static_cast<btScalar>(v[2]));
      }
      loaded = !pts.empty();
    }
  }
  if (!loaded || pts.empty()) {
    return nullptr;
  }

  // Compute optimized convex hull once and store only the minimal hull vertices.
  btConvexHullShape tmp_hull;
  for (const auto& p : pts) {
    tmp_hull.addPoint(p, false);
  }
  if (tmp_hull.getNumPoints() >= 4) {
    tmp_hull.optimizeConvexHull();
  }
  auto reduced = std::make_shared<std::vector<btVector3>>();
  const int n = tmp_hull.getNumPoints();
  reduced->reserve(static_cast<std::size_t>(n));
  const btVector3* hull_pts = tmp_hull.getUnscaledPoints();
  for (int i = 0; i < n; ++i) {
    reduced->push_back(hull_pts[i]);
  }

  {
    std::lock_guard<std::mutex> lk(cache_mu);
    cache[key] = reduced;
  }
  return reduced;
}

struct ShapeEntry {
  pinocchio::FrameIndex frame_id = 0;
  pinocchio::SE3 local_origin = pinocchio::SE3::Identity();
  std::unique_ptr<btConvexShape> shape;
  bool active = false;
};

std::unique_ptr<btConvexShape> build_bullet_shape_from_urdf(
    const ::urdf::GeometrySharedPtr& geom,
    const std::filesystem::path& urdf_dir,
    const std::vector<std::filesystem::path>& pkg_dirs) {
  if (!geom) return nullptr;

  if (geom->type == ::urdf::Geometry::MESH) {
    const auto* mesh = static_cast<const ::urdf::Mesh*>(geom.get());
    const auto resolved = resolve_mesh_path(mesh->filename, urdf_dir, pkg_dirs);
    if (resolved.empty()) return nullptr;
    const auto hull_verts = load_cached_convex_hull_vertices(
        resolved, mesh->scale.x, mesh->scale.y, mesh->scale.z);
    if (!hull_verts || hull_verts->empty()) return nullptr;
    auto hull = std::make_unique<btConvexHullShape>(
        reinterpret_cast<const btScalar*>(hull_verts->data()),
        static_cast<int>(hull_verts->size()),
        static_cast<int>(sizeof(btVector3)));
    hull->setMargin(kUrdfCollisionMargin);
    hull->recalcLocalAabb();
    return hull;
  }

  if (geom->type == ::urdf::Geometry::BOX) {
    const auto* box = static_cast<const ::urdf::Box*>(geom.get());
    auto shape = std::make_unique<btBoxShape>(btVector3(
        static_cast<btScalar>(0.5 * box->dim.x),
        static_cast<btScalar>(0.5 * box->dim.y),
        static_cast<btScalar>(0.5 * box->dim.z)));
    shape->setMargin(kUrdfCollisionMargin);
    return shape;
  }

  if (geom->type == ::urdf::Geometry::SPHERE) {
    const auto* sph = static_cast<const ::urdf::Sphere*>(geom.get());
    return std::make_unique<btSphereShape>(static_cast<btScalar>(sph->radius));
  }

  if (geom->type == ::urdf::Geometry::CYLINDER) {
    const auto* cyl = static_cast<const ::urdf::Cylinder*>(geom.get());
    auto shape = std::make_unique<btCylinderShapeZ>(btVector3(
        static_cast<btScalar>(cyl->radius),
        static_cast<btScalar>(cyl->radius),
        static_cast<btScalar>(0.5 * cyl->length)));
    shape->setMargin(kUrdfCollisionMargin);
    return shape;
  }

  return nullptr;
}

class CppCollisionGuard {
 public:
  CppCollisionGuard(
      const std::string& urdf_path,
      const std::unordered_map<std::string, double>& home_joints,
      const std::vector<std::pair<std::string, std::string>>& pair_names,
      double min_distance,
      double home_tolerance,
      double escape_epsilon,
      const std::optional<std::vector<double>>& explicit_clearances = std::nullopt)
      : min_distance_(min_distance),
        home_tolerance_(home_tolerance),
        escape_epsilon_(escape_epsilon) {
    if (explicit_clearances.has_value() &&
        explicit_clearances->size() != pair_names.size()) {
      throw std::invalid_argument("explicit_clearances length must match pair_names");
    }

    namespace fs = std::filesystem;
    const fs::path p = fs::absolute(fs::path(urdf_path));
    if (!fs::is_regular_file(p)) {
      throw std::runtime_error("URDF file not found: " + p.string());
    }

    auto urdf_tree = ::urdf::parseURDFFile(p.string());
    if (!urdf_tree) {
      throw std::runtime_error("Failed to parse URDF: " + p.string());
    }
    pinocchio::urdf::buildModel(urdf_tree, model_);
    data_ = pinocchio::Data(model_);

    const fs::path urdf_dir = p.parent_path();
    std::vector<fs::path> pkg_dirs;
    pkg_dirs.push_back(urdf_dir);
    if (urdf_dir.has_parent_path()) {
      pkg_dirs.push_back(urdf_dir.parent_path());
      if (urdf_dir.parent_path().has_parent_path()) {
        pkg_dirs.push_back(urdf_dir.parent_path().parent_path());
      }
    }

    // Map 1-DOF joints by name.
    q_home_ = pinocchio::neutral(model_);
    for (pinocchio::JointIndex jid = 1;
         jid < static_cast<pinocchio::JointIndex>(model_.njoints); ++jid) {
      if (model_.joints[jid].nq() == 1) {
        const std::string& jname = model_.names[jid];
        const int idx_q = model_.joints[jid].idx_q();
        joint_q_idx_[jname] = idx_q;
        joint_names_.push_back(jname);
        auto it = home_joints.find(jname);
        if (it != home_joints.end()) {
          q_home_[idx_q] = it->second;
        }
      }
    }
    q_buf_ = q_home_;

    // Identify which links are actually referenced by pair_names to skip building
    // unused meshes when possible (still track all links with collision tags).
    std::unordered_set<std::string> needed_links;
    needed_links.reserve(pair_names.size() * 2);
    for (const auto& [a, b] : pair_names) {
      needed_links.insert(a);
      needed_links.insert(b);
    }

    // Build Bullet3 convex shapes from URDF <collision> entries.
    for (const auto& [link_name, link_ptr] : urdf_tree->links_) {
      if (!link_ptr) continue;
      std::vector<::urdf::CollisionSharedPtr> colls;
      if (!link_ptr->collision_array.empty()) {
        colls = link_ptr->collision_array;
      } else if (link_ptr->collision) {
        colls.push_back(link_ptr->collision);
      }
      if (colls.empty()) continue;
      if (!model_.existFrame(link_name, pinocchio::BODY)) continue;
      const pinocchio::FrameIndex fid = model_.getFrameId(link_name, pinocchio::BODY);

      collision_link_names_.push_back(link_name);
      if (needed_links.find(link_name) == needed_links.end()) {
        continue;
      }

      for (const auto& coll : colls) {
        if (!coll || !coll->geometry) continue;
        auto shape = build_bullet_shape_from_urdf(coll->geometry, urdf_dir, pkg_dirs);
        if (!shape) continue;

        const int shape_idx = static_cast<int>(shapes_.size());
        link_to_shapes_[link_name].push_back(shape_idx);

        ShapeEntry entry;
        entry.frame_id = fid;
        entry.local_origin = urdf_pose_to_se3(coll->origin);
        entry.shape = std::move(shape);
        entry.active = false;
        shapes_.push_back(std::move(entry));
      }
    }
    std::sort(collision_link_names_.begin(), collision_link_names_.end());

    world_tf_.resize(shapes_.size());
    world_aabbs_.resize(shapes_.size());

    // Build protected pair table with warm-start separating axis per shape pair.
    pairs_.reserve(pair_names.size());
    std::unordered_set<pinocchio::FrameIndex> active_frame_set;
    for (std::size_t i = 0; i < pair_names.size(); ++i) {
      const auto& [name_a, name_b] = pair_names[i];
      auto it_a = link_to_shapes_.find(name_a);
      auto it_b = link_to_shapes_.find(name_b);
      if (it_a == link_to_shapes_.end() || it_b == link_to_shapes_.end()) {
        continue;
      }
      ProtectedPair pp;
      pp.name_a = name_a;
      pp.name_b = name_b;
      pp.clearance = explicit_clearances.has_value() ? (*explicit_clearances)[i] : min_distance_;
      for (int sa : it_a->second) {
        shapes_[static_cast<std::size_t>(sa)].active = true;
        active_frame_set.insert(shapes_[static_cast<std::size_t>(sa)].frame_id);
        for (int sb : it_b->second) {
          shapes_[static_cast<std::size_t>(sb)].active = true;
          active_frame_set.insert(shapes_[static_cast<std::size_t>(sb)].frame_id);
          pp.subpairs.push_back(ShapeSubPair{sa, sb, btVector3(0, 1, 0)});
        }
      }
      pairs_.push_back(std::move(pp));
    }

    active_frames_.assign(active_frame_set.begin(), active_frame_set.end());
    std::sort(active_frames_.begin(), active_frames_.end());
    for (std::size_t s = 0; s < shapes_.size(); ++s) {
      if (shapes_[s].active) {
        active_shape_indices_.push_back(static_cast<int>(s));
      }
    }

    // Calibrate clearances at home pose unless explicitly provided.
    if (!explicit_clearances.has_value()) {
      update_kinematics_and_aabbs(q_home_);
      for (auto& pp : pairs_) {
        const auto d = pair_distance_if_within(pp, min_distance_);
        pp.clearance = calibrated_clearance_cpp(min_distance_, d, home_tolerance_);
      }
    }
  }

  std::vector<std::tuple<std::string, std::string, double>> pair_clearances() const {
    std::vector<std::tuple<std::string, std::string, double>> out;
    out.reserve(pairs_.size());
    for (const auto& pp : pairs_) {
      out.emplace_back(pp.name_a, pp.name_b, pp.clearance);
    }
    return out;
  }

  std::vector<std::string> joint_names() const { return joint_names_; }

  std::vector<std::string> collision_link_names() const {
    return collision_link_names_;
  }

  std::size_t num_pairs() const { return pairs_.size(); }

  py::object evaluate(const py::dict& joint_values) {
    thread_local std::vector<std::pair<int, double>> updates;
    extract_updates(joint_values, updates);

    std::optional<CollisionHit> hit;
    {
      py::gil_scoped_release release;
      std::lock_guard<std::mutex> lock(mu_);
      q_buf_ = q_home_;
      for (const auto& kv : updates) {
        q_buf_[kv.first] = kv.second;
      }
      hit = evaluate_q_locked(q_buf_);
    }
    return hit_to_py(hit);
  }

  py::object evaluate_overlay(const py::dict& base_values,
                              const py::dict& overlay_values) {
    thread_local std::vector<std::pair<int, double>> base_updates;
    thread_local std::vector<std::pair<int, double>> overlay_updates;
    extract_updates(base_values, base_updates);
    extract_updates(overlay_values, overlay_updates);

    std::optional<CollisionHit> hit;
    {
      py::gil_scoped_release release;
      std::lock_guard<std::mutex> lock(mu_);
      q_buf_ = q_home_;
      for (const auto& kv : base_updates) {
        q_buf_[kv.first] = kv.second;
      }
      for (const auto& kv : overlay_updates) {
        q_buf_[kv.first] = kv.second;
      }
      hit = evaluate_q_locked(q_buf_);
    }
    return hit_to_py(hit);
  }

  py::tuple check(const py::dict& measured, const py::dict& command) {
    thread_local std::vector<std::pair<int, double>> meas_updates;
    thread_local std::vector<std::pair<int, double>> cmd_updates;
    extract_updates(measured, meas_updates);
    extract_updates(command, cmd_updates);

    std::optional<CollisionHit> current;
    std::optional<CollisionHit> candidate;
    bool allowed = true;
    {
      py::gil_scoped_release release;
      std::lock_guard<std::mutex> lock(mu_);
      q_buf_ = q_home_;
      for (const auto& kv : meas_updates) {
        q_buf_[kv.first] = kv.second;
      }
      current = evaluate_q_locked(q_buf_);

      bool changed = false;
      for (const auto& kv : cmd_updates) {
        if (q_buf_[kv.first] != kv.second) {
          q_buf_[kv.first] = kv.second;
          changed = true;
        }
      }
      if (changed) {
        candidate = evaluate_q_locked(q_buf_);
      } else {
        candidate = current;
      }

      if (!candidate.has_value()) {
        allowed = true;
      } else if (!current.has_value()) {
        allowed = false;
      } else {
        allowed = (candidate->deficit() <= current->deficit() - escape_epsilon_);
      }
    }
    return py::make_tuple(allowed, hit_to_py(candidate), current.has_value());
  }

 private:
  struct ShapeSubPair {
    int shape_a;
    int shape_b;
    btVector3 cached_sep_axis;
  };

  struct ProtectedPair {
    std::string name_a;
    std::string name_b;
    std::vector<ShapeSubPair> subpairs;
    double clearance = 0.02;
  };

  void extract_updates(const py::dict& values,
                       std::vector<std::pair<int, double>>& out) const {
    out.clear();
    if (out.capacity() < values.size()) {
      out.reserve(values.size());
    }
    for (auto item : values) {
      PyObject* k_ptr = item.first.ptr();
      if (!PyUnicode_Check(k_ptr)) continue;
      Py_ssize_t klen = 0;
      const char* kstr = PyUnicode_AsUTF8AndSize(k_ptr, &klen);
      if (kstr == nullptr) {
        throw py::error_already_set();
      }
      const std::string_view sv(kstr, static_cast<std::size_t>(klen));
      auto it = joint_q_idx_.find(sv);
      if (it == joint_q_idx_.end()) continue;
      const double val = PyFloat_AsDouble(item.second.ptr());
      if (val == -1.0 && PyErr_Occurred()) {
        throw py::error_already_set();
      }
      out.emplace_back(it->second, val);
    }
  }

  static py::object hit_to_py(const std::optional<CollisionHit>& hit) {
    if (!hit.has_value()) {
      return py::none();
    }
    return py::make_tuple(hit->link_a, hit->link_b, hit->distance, hit->clearance);
  }

  void update_kinematics_and_aabbs(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model_, data_, q);
    for (pinocchio::FrameIndex fid : active_frames_) {
      pinocchio::updateFramePlacement(model_, data_, fid);
    }

    for (int sidx : active_shape_indices_) {
      const auto& entry = shapes_[static_cast<std::size_t>(sidx)];
      const pinocchio::SE3 oMg = data_.oMf[entry.frame_id] * entry.local_origin;
      const btTransform tf = se3_to_bt(oMg);
      world_tf_[static_cast<std::size_t>(sidx)] = tf;
      entry.shape->getAabb(tf,
                           world_aabbs_[static_cast<std::size_t>(sidx)].lo,
                           world_aabbs_[static_cast<std::size_t>(sidx)].hi);
    }
  }

  std::optional<double> pair_distance_if_within(ProtectedPair& pp,
                                                double clearance) {
    const double clearance_sq =
        clearance > 0.0 ? (clearance * clearance) : 0.0;
    bool any_hit = false;
    double best = std::numeric_limits<double>::infinity();

    for (auto& sp : pp.subpairs) {
      const auto& aabb_a = world_aabbs_[static_cast<std::size_t>(sp.shape_a)];
      const auto& aabb_b = world_aabbs_[static_cast<std::size_t>(sp.shape_b)];
      if (aabb_dist_sq(aabb_a, aabb_b) > clearance_sq) {
        continue;
      }

      const auto* shape_a = shapes_[static_cast<std::size_t>(sp.shape_a)].shape.get();
      const auto* shape_b = shapes_[static_cast<std::size_t>(sp.shape_b)].shape.get();
      const auto& tf_a = world_tf_[static_cast<std::size_t>(sp.shape_a)];
      const auto& tf_b = world_tf_[static_cast<std::size_t>(sp.shape_b)];

      simplex_.reset();
      btGjkPairDetector detector(shape_a, shape_b, &simplex_, &epa_);
      detector.setCachedSeparatingAxis(sp.cached_sep_axis);

      btDiscreteCollisionDetectorInterface::ClosestPointInput input;
      input.m_transformA = tf_a;
      input.m_transformB = tf_b;
      const btScalar max_d =
          static_cast<btScalar>(std::max(0.0, clearance)) +
          shape_a->getMargin() + shape_b->getMargin() + btScalar(1e-3);
      input.m_maximumDistanceSquared = max_d * max_d;

      MinDistanceResult result;
      detector.getClosestPoints(input, result, nullptr);
      const btVector3 axis = detector.getCachedSeparatingAxis();
      if (axis.length2() > btScalar(1e-12)) {
        sp.cached_sep_axis = axis;
      }
      if (result.found) {
        const double d = static_cast<double>(result.min_distance);
        if (d <= clearance && (!any_hit || d < best)) {
          any_hit = true;
          best = d;
        }
      }
    }
    if (!any_hit) return std::nullopt;
    return best;
  }

  std::optional<CollisionHit> evaluate_q_locked(const Eigen::VectorXd& q) {
    update_kinematics_and_aabbs(q);
    std::optional<CollisionHit> worst;
    for (auto& pp : pairs_) {
      const auto dist = pair_distance_if_within(pp, pp.clearance);
      if (dist.has_value() && *dist < pp.clearance) {
        CollisionHit candidate{pp.name_a, pp.name_b, *dist, pp.clearance};
        if (!worst.has_value() || candidate.deficit() > worst->deficit()) {
          worst = std::move(candidate);
        }
      }
    }
    return worst;
  }

  pinocchio::Model model_;
  pinocchio::Data data_;

  Eigen::VectorXd q_home_;
  Eigen::VectorXd q_buf_;
  std::map<std::string, int, std::less<>> joint_q_idx_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> collision_link_names_;

  std::vector<ShapeEntry> shapes_;
  std::unordered_map<std::string, std::vector<int>> link_to_shapes_;
  std::vector<pinocchio::FrameIndex> active_frames_;
  std::vector<int> active_shape_indices_;
  std::vector<btTransform> world_tf_;
  std::vector<WorldAabb> world_aabbs_;
  std::vector<ProtectedPair> pairs_;

  btVoronoiSimplexSolver simplex_;
  btGjkEpaPenetrationDepthSolver epa_;

  double min_distance_ = 0.02;
  double home_tolerance_ = 0.008;
  double escape_epsilon_ = 1e-5;
  std::mutex mu_;
};

}  // namespace

PYBIND11_MODULE(_collision_guard_cpp, m) {
  m.doc() = "C++17 Pinocchio + Bullet3 (float64) native self-collision guard";

  py::class_<CppCollisionGuard>(m, "CppCollisionGuard")
      .def(py::init<const std::string&,
                    const std::unordered_map<std::string, double>&,
                    const std::vector<std::pair<std::string, std::string>>&,
                    double,
                    double,
                    double,
                    const std::optional<std::vector<double>>&>(),
           py::arg("urdf_path"),
           py::arg("home_joints"),
           py::arg("pair_names"),
           py::arg("min_distance") = 0.02,
           py::arg("home_tolerance") = 0.008,
           py::arg("escape_epsilon") = 1e-5,
           py::arg("explicit_clearances") = std::nullopt)
      .def("evaluate", &CppCollisionGuard::evaluate, py::arg("joint_values"))
      .def("evaluate_overlay", &CppCollisionGuard::evaluate_overlay,
           py::arg("base_values"), py::arg("overlay_values"))
      .def("check", &CppCollisionGuard::check,
           py::arg("measured"), py::arg("command"))
      .def("pair_clearances", &CppCollisionGuard::pair_clearances)
      .def("joint_names", &CppCollisionGuard::joint_names)
      .def("collision_link_names", &CppCollisionGuard::collision_link_names)
      .def_property_readonly("num_pairs", &CppCollisionGuard::num_pairs);
}
