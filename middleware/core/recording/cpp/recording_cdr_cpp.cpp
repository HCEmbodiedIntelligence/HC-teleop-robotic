#include <pybind11/pybind11.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace py = pybind11;

namespace {

inline uint32_t read_u32(const uint8_t* p, bool little_endian) {
  uint32_t v;
  std::memcpy(&v, p, 4);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  if (!little_endian) v = __builtin_bswap32(v);
#else
  if (little_endian) v = __builtin_bswap32(v);
#endif
  return v;
}

inline void write_u32(uint8_t* p, uint32_t v, bool little_endian) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  if (!little_endian) v = __builtin_bswap32(v);
#else
  if (little_endian) v = __builtin_bswap32(v);
#endif
  std::memcpy(p, &v, 4);
}

inline double read_f64(const uint8_t* p, bool little_endian) {
  uint64_t bits;
  std::memcpy(&bits, p, 8);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  if (!little_endian) bits = __builtin_bswap64(bits);
#else
  if (little_endian) bits = __builtin_bswap64(bits);
#endif
  double d;
  std::memcpy(&d, &bits, 8);
  return d;
}

inline void write_f64(uint8_t* p, double d, bool little_endian) {
  uint64_t bits;
  std::memcpy(&bits, &d, 8);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  if (!little_endian) bits = __builtin_bswap64(bits);
#else
  if (little_endian) bits = __builtin_bswap64(bits);
#endif
  std::memcpy(p, &bits, 8);
}

struct BytesView {
  const uint8_t* data;
  std::size_t size;
};

BytesView require_cdr_bytes(const py::handle& obj) {
  if (!PyBytes_Check(obj.ptr())) {
    throw py::value_error("unsupported CDR representation");
  }
  char* buf = nullptr;
  Py_ssize_t len = 0;
  if (PyBytes_AsStringAndSize(obj.ptr(), &buf, &len) != 0) {
    throw py::error_already_set();
  }
  const auto* ubuf = reinterpret_cast<const uint8_t*>(buf);
  const auto ulen = static_cast<std::size_t>(len);
  if (ulen < 4 || ubuf[0] != 0 || (ubuf[1] != 0 && ubuf[1] != 1)) {
    throw py::value_error("unsupported CDR representation");
  }
  return {ubuf, ulen};
}

class CdrCursor {
 public:
  CdrCursor(const uint8_t* data, std::size_t size)
      : data_(data), size_(size), offset_(4), little_endian_(data[1] != 0) {}

  std::size_t offset() const { return offset_; }
  void set_offset(std::size_t off) { offset_ = off; }
  bool little_endian() const { return little_endian_; }

  void align(std::size_t alignment) {
    const std::size_t rem = (offset_ - 4) % alignment;
    if (rem != 0) {
      offset_ += alignment - rem;
    }
  }

  uint32_t read_uint() {
    align(4);
    if (offset_ + 4 > size_) {
      throw py::value_error("truncated CDR length");
    }
    const uint32_t val = read_u32(data_ + offset_, little_endian_);
    offset_ += 4;
    return val;
  }

  py::str read_py_string() {
    const uint32_t count = read_uint();
    const std::size_t end = offset_ + static_cast<std::size_t>(count);
    if (count == 0 || end > size_ || data_[end - 1] != 0) {
      throw py::value_error("invalid CDR string");
    }
    PyObject* py_s = PyUnicode_DecodeUTF8(
        reinterpret_cast<const char*>(data_ + offset_),
        static_cast<Py_ssize_t>(count - 1), nullptr);
    if (py_s == nullptr) {
      throw py::error_already_set();
    }
    offset_ = end;
    return py::reinterpret_steal<py::str>(py_s);
  }

  void skip_string() {
    const uint32_t count = read_uint();
    const std::size_t end = offset_ + static_cast<std::size_t>(count);
    if (count == 0 || end > size_ || data_[end - 1] != 0) {
      throw py::value_error("invalid CDR string");
    }
    // Validate UTF-8 on frame_id just like Python .decode('utf-8').
    PyObject* py_s = PyUnicode_DecodeUTF8(
        reinterpret_cast<const char*>(data_ + offset_),
        static_cast<Py_ssize_t>(count - 1), nullptr);
    if (py_s == nullptr) {
      throw py::error_already_set();
    }
    Py_DECREF(py_s);
    offset_ = end;
  }

  py::tuple read_doubles_tuple(uint32_t& out_count) {
    const uint32_t count = read_uint();
    out_count = count;
    if (count > 0) {
      align(8);
    }
    const std::size_t bytes_needed = static_cast<std::size_t>(count) * 8;
    if (bytes_needed > size_ - offset_) {
      throw py::value_error("truncated CDR double sequence");
    }
    py::tuple out(count);
    for (uint32_t i = 0; i < count; ++i) {
      out[i] = py::float_(read_f64(data_ + offset_ + static_cast<std::size_t>(i) * 8,
                                   little_endian_));
    }
    offset_ += bytes_needed;
    return out;
  }

  uint32_t skip_doubles() {
    const uint32_t count = read_uint();
    if (count > 0) {
      align(8);
    }
    const std::size_t bytes_needed = static_cast<std::size_t>(count) * 8;
    if (bytes_needed > size_ - offset_) {
      throw py::value_error("truncated CDR double sequence");
    }
    offset_ += bytes_needed;
    return count;
  }

 private:
  const uint8_t* data_;
  std::size_t size_;
  std::size_t offset_;
  bool little_endian_;
};

py::str read_string_py(py::object raw) {
  const auto view = require_cdr_bytes(raw);
  CdrCursor cursor(view.data, view.size);
  return cursor.read_py_string();
}

py::bytes write_string_py(py::str value) {
  Py_ssize_t utf8_len = 0;
  const char* utf8 = PyUnicode_AsUTF8AndSize(value.ptr(), &utf8_len);
  if (utf8 == nullptr) {
    throw py::error_already_set();
  }
  const uint32_t count = static_cast<uint32_t>(utf8_len + 1);
  std::string out(8 + count, '\0');
  auto* dst = reinterpret_cast<uint8_t*>(out.data());
  dst[0] = 0x00;
  dst[1] = 0x01;
  dst[2] = 0x00;
  dst[3] = 0x00;
  write_u32(dst + 4, count, true);
  if (utf8_len > 0) {
    std::memcpy(dst + 8, utf8, static_cast<std::size_t>(utf8_len));
  }
  dst[8 + utf8_len] = 0;
  return py::bytes(out);
}

py::tuple parse_joint_state_py(py::object raw) {
  const auto view = require_cdr_bytes(raw);
  if (view.size < 12) {
    throw py::value_error("truncated CDR Header");
  }
  CdrCursor cursor(view.data, view.size);
  cursor.set_offset(12);
  cursor.skip_string();
  const std::size_t prefix_len = cursor.offset();

  const uint32_t count = cursor.read_uint();
  if (count > (view.size - cursor.offset()) / 5) {
    throw py::value_error("impossible CDR joint count");
  }
  py::tuple names(count);
  for (uint32_t i = 0; i < count; ++i) {
    names[i] = cursor.read_py_string();
  }
  uint32_t pos_count = 0;
  py::tuple positions = cursor.read_doubles_tuple(pos_count);
  const uint32_t vel_count = cursor.skip_doubles();
  const uint32_t eff_count = cursor.skip_doubles();

  const std::size_t padding_budget =
      4 * (static_cast<std::size_t>(pos_count == 0) +
           static_cast<std::size_t>(vel_count == 0) +
           static_cast<std::size_t>(eff_count == 0));
  if (cursor.offset() > view.size ||
      view.size - cursor.offset() > padding_budget) {
    throw py::value_error("unexpected trailing JointState data");
  }
  py::bytes prefix(reinterpret_cast<const char*>(view.data), prefix_len);
  return py::make_tuple(names, positions, prefix, cursor.offset());
}

py::bytes trim_joint_state_cdr_py(py::object raw) {
  const auto view = require_cdr_bytes(raw);
  if (view.size < 12) {
    throw py::value_error("truncated CDR Header");
  }
  CdrCursor cursor(view.data, view.size);
  cursor.set_offset(12);
  cursor.skip_string();

  const uint32_t count = cursor.read_uint();
  if (count > (view.size - cursor.offset()) / 5) {
    throw py::value_error("impossible CDR joint count");
  }
  for (uint32_t i = 0; i < count; ++i) {
    cursor.skip_string();
  }
  const uint32_t pos_count = cursor.skip_doubles();
  const uint32_t vel_count = cursor.skip_doubles();
  const uint32_t eff_count = cursor.skip_doubles();

  const std::size_t padding_budget =
      4 * (static_cast<std::size_t>(pos_count == 0) +
           static_cast<std::size_t>(vel_count == 0) +
           static_cast<std::size_t>(eff_count == 0));
  if (cursor.offset() > view.size ||
      view.size - cursor.offset() > padding_budget) {
    throw py::value_error("unexpected trailing JointState data");
  }
  if (cursor.offset() == view.size) {
    return py::reinterpret_borrow<py::bytes>(raw);
  }
  return py::bytes(reinterpret_cast<const char*>(view.data), cursor.offset());
}

py::bytes position_label_py(py::object header_prefix,
                            py::sequence names,
                            py::sequence positions) {
  const auto prefix = require_cdr_bytes(header_prefix);
  const std::size_t n = static_cast<std::size_t>(py::len(names));
  if (n != static_cast<std::size_t>(py::len(positions))) {
    throw py::value_error("joint names/positions length mismatch");
  }
  const bool little_endian = (prefix.data[1] != 0);

  std::vector<const char*> utf8_ptrs;
  std::vector<std::size_t> utf8_lens;
  std::vector<py::str> keep_alive;
  utf8_ptrs.reserve(n);
  utf8_lens.reserve(n);
  keep_alive.reserve(n);

  auto align_up = [](std::size_t len, std::size_t alignment) -> std::size_t {
    const std::size_t rem = (len - 4) % alignment;
    return rem == 0 ? len : len + (alignment - rem);
  };

  std::size_t total_size = align_up(prefix.size, 4) + 4;  // name count
  for (std::size_t i = 0; i < n; ++i) {
    py::str s = names[i].cast<py::str>();
    Py_ssize_t slen = 0;
    const char* ptr = PyUnicode_AsUTF8AndSize(s.ptr(), &slen);
    if (ptr == nullptr) {
      throw py::error_already_set();
    }
    keep_alive.push_back(std::move(s));
    utf8_ptrs.push_back(ptr);
    utf8_lens.push_back(static_cast<std::size_t>(slen));
    total_size = align_up(total_size, 4) + 4 + static_cast<std::size_t>(slen) + 1;
  }
  total_size = align_up(total_size, 4) + 4;  // position count
  if (n > 0) {
    total_size = align_up(total_size, 8) + n * 8;
  }
  total_size = align_up(total_size, 4) + 4;  // velocity count = 0
  total_size = align_up(total_size, 4) + 4;  // effort count = 0

  std::string out(total_size, '\0');
  auto* dst = reinterpret_cast<uint8_t*>(out.data());
  std::memcpy(dst, prefix.data, prefix.size);
  std::size_t pos = align_up(prefix.size, 4);

  write_u32(dst + pos, static_cast<uint32_t>(n), little_endian);
  pos += 4;
  for (std::size_t i = 0; i < n; ++i) {
    pos = align_up(pos, 4);
    const uint32_t slen_with_nul = static_cast<uint32_t>(utf8_lens[i] + 1);
    write_u32(dst + pos, slen_with_nul, little_endian);
    pos += 4;
    if (utf8_lens[i] > 0) {
      std::memcpy(dst + pos, utf8_ptrs[i], utf8_lens[i]);
      pos += utf8_lens[i];
    }
    dst[pos++] = 0;
  }

  pos = align_up(pos, 4);
  write_u32(dst + pos, static_cast<uint32_t>(n), little_endian);
  pos += 4;
  if (n > 0) {
    pos = align_up(pos, 8);
    for (std::size_t i = 0; i < n; ++i) {
      const double v = positions[i].cast<double>();
      write_f64(dst + pos, v, little_endian);
      pos += 8;
    }
  }
  pos = align_up(pos, 4);
  write_u32(dst + pos, 0U, little_endian);
  pos += 4;
  pos = align_up(pos, 4);
  write_u32(dst + pos, 0U, little_endian);

  return py::bytes(out);
}

// Fast double-sequence extraction supporting array.array('d'), memoryview,
// list, tuple, or any Python sequence — zero NumPy/rclpy dependency.
void extract_doubles(const py::handle& obj, std::vector<double>& out) {
  out.clear();
  PyObject* ptr = obj.ptr();
  if (PyObject_CheckBuffer(ptr)) {
    Py_buffer view;
    if (PyObject_GetBuffer(ptr, &view, PyBUF_FORMAT | PyBUF_CONTIG_RO) == 0) {
      if (view.ndim == 1 && view.format != nullptr && view.format[0] == 'd' &&
          view.format[1] == '\0' && view.itemsize == sizeof(double)) {
        const std::size_t n = static_cast<std::size_t>(view.len) / sizeof(double);
        out.resize(n);
        if (n > 0) {
          std::memcpy(out.data(), view.buf, n * sizeof(double));
        }
        PyBuffer_Release(&view);
        return;
      }
      PyBuffer_Release(&view);
    } else {
      PyErr_Clear();
    }
  }
  if (PyList_CheckExact(ptr)) {
    const Py_ssize_t n = PyList_GET_SIZE(ptr);
    out.resize(static_cast<std::size_t>(n));
    for (Py_ssize_t i = 0; i < n; ++i) {
      const double v = PyFloat_AsDouble(PyList_GET_ITEM(ptr, i));
      if (v == -1.0 && PyErr_Occurred()) {
        throw py::error_already_set();
      }
      out[static_cast<std::size_t>(i)] = v;
    }
    return;
  }
  if (PyTuple_CheckExact(ptr)) {
    const Py_ssize_t n = PyTuple_GET_SIZE(ptr);
    out.resize(static_cast<std::size_t>(n));
    for (Py_ssize_t i = 0; i < n; ++i) {
      const double v = PyFloat_AsDouble(PyTuple_GET_ITEM(ptr, i));
      if (v == -1.0 && PyErr_Occurred()) {
        throw py::error_already_set();
      }
      out[static_cast<std::size_t>(i)] = v;
    }
    return;
  }
  py::sequence seq = py::reinterpret_borrow<py::sequence>(obj);
  const std::size_t n = static_cast<std::size_t>(py::len(seq));
  out.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    out[i] = seq[i].cast<double>();
  }
}

py::bytes serialize_joint_state_py(int64_t sec,
                                   int64_t nanosec,
                                   py::str frame_id,
                                   py::sequence names,
                                   py::object positions,
                                   py::object velocities,
                                   py::object efforts) {
  if (sec < INT32_MIN || sec > INT32_MAX) {
    throw py::value_error("JointState header.stamp.sec out of int32 range");
  }
  if (nanosec < 0 || nanosec > UINT32_MAX) {
    throw py::value_error("JointState header.stamp.nanosec out of uint32 range");
  }

  Py_ssize_t fid_len = 0;
  const char* fid_utf8 = PyUnicode_AsUTF8AndSize(frame_id.ptr(), &fid_len);
  if (fid_utf8 == nullptr) {
    throw py::error_already_set();
  }

  const std::size_t n_names = static_cast<std::size_t>(py::len(names));
  std::vector<const char*> name_ptrs;
  std::vector<std::size_t> name_lens;
  std::vector<py::str> keep_alive;
  name_ptrs.reserve(n_names);
  name_lens.reserve(n_names);
  keep_alive.reserve(n_names);

  for (std::size_t i = 0; i < n_names; ++i) {
    py::str s = names[i].cast<py::str>();
    Py_ssize_t slen = 0;
    const char* ptr = PyUnicode_AsUTF8AndSize(s.ptr(), &slen);
    if (ptr == nullptr) {
      throw py::error_already_set();
    }
    keep_alive.push_back(std::move(s));
    name_ptrs.push_back(ptr);
    name_lens.push_back(static_cast<std::size_t>(slen));
  }

  std::vector<double> pos_vec, vel_vec, eff_vec;
  extract_doubles(positions, pos_vec);
  extract_doubles(velocities, vel_vec);
  extract_doubles(efforts, eff_vec);

  auto align_up = [](std::size_t len, std::size_t alignment) -> std::size_t {
    const std::size_t rem = (len - 4) % alignment;
    return rem == 0 ? len : len + (alignment - rem);
  };

  // Compute exact trimmed CDR1 size (no trailing slack for empty arrays).
  std::size_t total = 12;  // 4-byte encapsulation + int32 sec + uint32 nanosec
  total += 4 + static_cast<std::size_t>(fid_len) + 1;  // frame_id
  total = align_up(total, 4) + 4;                      // name count
  for (std::size_t i = 0; i < n_names; ++i) {
    total = align_up(total, 4) + 4 + name_lens[i] + 1;
  }
  auto add_double_seq = [&](std::size_t count) {
    total = align_up(total, 4) + 4;
    if (count > 0) {
      total = align_up(total, 8) + count * 8;
    }
  };
  add_double_seq(pos_vec.size());
  add_double_seq(vel_vec.size());
  add_double_seq(eff_vec.size());

  std::string out(total, '\0');
  auto* dst = reinterpret_cast<uint8_t*>(out.data());
  dst[0] = 0x00;
  dst[1] = 0x01;  // CDR_LE
  dst[2] = 0x00;
  dst[3] = 0x00;

  const uint32_t sec_bits = static_cast<uint32_t>(static_cast<int32_t>(sec));
  const uint32_t nsec_bits = static_cast<uint32_t>(nanosec);
  write_u32(dst + 4, sec_bits, true);
  write_u32(dst + 8, nsec_bits, true);

  std::size_t pos = 12;
  write_u32(dst + pos, static_cast<uint32_t>(fid_len + 1), true);
  pos += 4;
  if (fid_len > 0) {
    std::memcpy(dst + pos, fid_utf8, static_cast<std::size_t>(fid_len));
    pos += static_cast<std::size_t>(fid_len);
  }
  dst[pos++] = 0;

  pos = align_up(pos, 4);
  write_u32(dst + pos, static_cast<uint32_t>(n_names), true);
  pos += 4;
  for (std::size_t i = 0; i < n_names; ++i) {
    pos = align_up(pos, 4);
    write_u32(dst + pos, static_cast<uint32_t>(name_lens[i] + 1), true);
    pos += 4;
    if (name_lens[i] > 0) {
      std::memcpy(dst + pos, name_ptrs[i], name_lens[i]);
      pos += name_lens[i];
    }
    dst[pos++] = 0;
  }

  auto write_double_seq = [&](const std::vector<double>& vec) {
    pos = align_up(pos, 4);
    write_u32(dst + pos, static_cast<uint32_t>(vec.size()), true);
    pos += 4;
    if (!vec.empty()) {
      pos = align_up(pos, 8);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      std::memcpy(dst + pos, vec.data(), vec.size() * sizeof(double));
      pos += vec.size() * sizeof(double);
#else
      for (double v : vec) {
        write_f64(dst + pos, v, true);
        pos += 8;
      }
#endif
    }
  };
  write_double_seq(pos_vec);
  write_double_seq(vel_vec);
  write_double_seq(eff_vec);

  return py::bytes(out);
}

}  // namespace

PYBIND11_MODULE(_recording_cdr_cpp, m) {
  m.doc() = "C++17 zero-dependency CDR1 codec for HC-teleop recording labels";

  m.def("read_string", &read_string_py, py::arg("raw"));
  m.def("write_string", &write_string_py, py::arg("value"));
  m.def("parse_joint_state", &parse_joint_state_py, py::arg("raw"));
  m.def("trim_joint_state_cdr", &trim_joint_state_cdr_py, py::arg("raw"));
  m.def("position_label", &position_label_py, py::arg("header_prefix"),
        py::arg("names"), py::arg("positions"));
  m.def("serialize_joint_state", &serialize_joint_state_py,
        py::arg("sec"), py::arg("nanosec"), py::arg("frame_id"),
        py::arg("names"), py::arg("positions"),
        py::arg("velocities") = py::tuple(),
        py::arg("efforts") = py::tuple());
}

