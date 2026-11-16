// Validated model arrays enter this library once; FK, geometric Jacobians and
// bounded DLS execute natively in an isolated worker, never the control loop.
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <new>

#define HC_EXPORT extern "C" __attribute__((visibility("default")))

namespace {
constexpr int kMaxJoints=256;
constexpr int kMaxActive=64;
using Matrix=Eigen::Matrix4d;
using Vector=Eigen::Vector3d;
using Clock=std::chrono::steady_clock;
using MatrixRow=Eigen::Matrix<double,4,4,Eigen::RowMajor>;
struct Joint {
    int parent=0,child=0,kind=0;
    Matrix origin=Matrix::Identity();
    Vector axis=Vector::UnitX();
    double lower=0,upper=0;
};
struct Model {
    int count=0,links=0,root=0;
    std::array<Joint,kMaxJoints> joints;
    std::array<int,kMaxJoints+1> parent_joint;
};
struct Frames {
    std::array<Matrix,kMaxJoints+1> links;
    std::array<Matrix,kMaxJoints> joints;
};

bool forward(const Model& m,const double* q,Frames& f) {
    f.links[m.root].setIdentity();
    for(int i=0;i<m.count;++i) {
        if(!std::isfinite(q[i]))return false;
        const auto& j=m.joints[i];
        f.joints[i]=f.links[j.parent]*j.origin;
        Matrix motion=Matrix::Identity();
        if(j.kind==1)motion.topLeftCorner<3,3>()=Eigen::AngleAxisd(q[i],j.axis).toRotationMatrix();
        if(j.kind==2)motion.topRightCorner<3,1>()=j.axis*q[i];
        f.links[j.child]=f.joints[i]*motion;
    }
    return true;
}
bool frame_valid(const Model& m,int tip,int base) {
    return tip>=0&&tip<m.links&&base>=-1&&base<m.links;
}
Matrix relative(const Frames& f,int tip,int base) {
    return base<0?f.links[tip]:Matrix(f.links[base].inverse()*f.links[tip]);
}
bool chain_valid(const Model& m,int tip,int base,const int* active,int n) {
    std::array<bool,kMaxJoints> chain{};
    int link=tip;
    while(link!=(base<0?m.root:base)) {
        const int index=m.parent_joint[link];
        if(index<0)return false;
        chain[index]=true;link=m.joints[index].parent;
    }
    for(int i=0;i<n;++i) {
        if(active[i]<0||active[i]>=m.count||!chain[active[i]]||m.joints[active[i]].kind==0)return false;
        for(int j=0;j<i;++j)if(active[i]==active[j])return false;
    }
    return true;
}
void jacobian(const Model& m,const Frames& f,int tip,int base,const int* active,int n,
              Eigen::Matrix<double,6,kMaxActive>& out) {
    const Vector p=f.links[tip].topRightCorner<3,1>();
    const Eigen::Matrix3d rotate=base<0?Eigen::Matrix3d::Identity():Eigen::Matrix3d(f.links[base].topLeftCorner<3,3>().transpose());
    out.setZero();
    for(int i=0;i<n;++i) {
        const int index=active[i];const auto& j=m.joints[index];
        const Vector axis=f.joints[index].topLeftCorner<3,3>()*j.axis;
        if(j.kind==1) {
            out.block<3,1>(0,i)=rotate*axis.cross(p-f.joints[index].topRightCorner<3,1>());
            out.block<3,1>(3,i)=rotate*axis;
        } else out.block<3,1>(0,i)=rotate*axis;
    }
}
double elapsed(const Clock::time_point& start) {
    return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
}

HC_EXPORT int hc_kinematics_abi_version() {return 1;}

HC_EXPORT void* hc_model_create(int count,int links,int root,const int* parents,const int* children,
    const int* kinds,const double* origins,const double* axes,const double* lower,const double* upper) {
    if(count<1||count>kMaxJoints||links!=count+1||root<0||root>=links||!parents||!children||!kinds||!origins||!axes||!lower||!upper)return nullptr;
    auto* m=new(std::nothrow) Model;
    if(!m)return nullptr;
    m->count=count;m->links=links;m->root=root;m->parent_joint.fill(-1);
    std::array<bool,kMaxJoints+1> visited{};visited[root]=true;
    for(int i=0;i<count;++i) {
        auto& j=m->joints[i];j.parent=parents[i];j.child=children[i];j.kind=kinds[i];
        if(j.parent<0||j.parent>=links||j.child<0||j.child>=links||j.kind<0||j.kind>2||!visited[j.parent]||visited[j.child]) {delete m;return nullptr;}
        j.origin=Eigen::Map<const MatrixRow>(origins+16*i);
        j.axis=Eigen::Map<const Vector>(axes+3*i);
        j.lower=lower[i];j.upper=upper[i];
        if(!j.origin.allFinite()||!j.axis.allFinite()||j.axis.norm()<1e-9||std::isnan(j.lower)||std::isnan(j.upper)||j.lower>j.upper) {delete m;return nullptr;}
        j.axis.normalize();visited[j.child]=true;m->parent_joint[j.child]=i;
    }
    return m;
}

HC_EXPORT void hc_model_destroy(void* model) {delete static_cast<Model*>(model);}

HC_EXPORT int hc_fk(void* model,const double* q,int tip,int base,double* result) {
    if(!model||!q||!result)return -1;
    const auto& m=*static_cast<Model*>(model);
    if(!frame_valid(m,tip,base))return -1;
    Frames frames;if(!forward(m,q,frames))return -1;
    Eigen::Map<MatrixRow> mapped(result);mapped=relative(frames,tip,base);
    return 0;
}

HC_EXPORT int hc_jacobian(void* model,const double* q,int tip,int base,const int* active,int n,double* result) {
    if(!model||!q||!active||!result||n<1||n>kMaxActive)return -1;
    const auto& m=*static_cast<Model*>(model);
    if(!frame_valid(m,tip,base)||!chain_valid(m,tip,base,active,n))return -1;
    Frames frames;if(!forward(m,q,frames))return -1;
    Eigen::Matrix<double,6,kMaxActive> j;jacobian(m,frames,tip,base,active,n,j);
    for(int row=0;row<6;++row)for(int col=0;col<n;++col)result[row*n+col]=j(row,col);
    return 0;
}

// stats = position_error, orientation_error, iterations, elapsed_ms, reason.
// reason: 0 converged, 1 iteration limit, 2 deadline. Negative return is invalid.
HC_EXPORT int hc_ik(void* model,const double* seed,int tip,int base,const int* active,int n,
    const double* target_data,int max_iterations,double timeout_ms,double damping,double max_step,
    double position_tolerance,double orientation_tolerance,double orientation_weight,double* result,double* stats) {
    if(!model||!seed||!active||!target_data||!result||!stats||n<1||n>kMaxActive||max_iterations<1||max_iterations>200)return -1;
    const auto& m=*static_cast<Model*>(model);
    if(!frame_valid(m,tip,base)||!chain_valid(m,tip,base,active,n))return -1;
    for(double value:{timeout_ms,damping,max_step,position_tolerance,orientation_tolerance,orientation_weight})if(!std::isfinite(value)||value<0)return -1;
    if(timeout_ms==0||damping==0||max_step==0)return -1;
    const Matrix target=Eigen::Map<const MatrixRow>(target_data);
    if(!target.allFinite())return -1;
    for(int i=0;i<m.count;++i) {if(!std::isfinite(seed[i]))return -1;result[i]=seed[i];}
    const auto start=Clock::now();
    Frames frames;
    Eigen::Matrix<double,6,kMaxActive> j;
    Eigen::Matrix<double,6,1> error,weighted;
    Eigen::Matrix<double,kMaxActive,1> delta;
    int reason=1,iteration=0;
    for(;iteration<=max_iterations;++iteration) {
        if(!forward(m,result,frames))return -1;
        const Matrix current=relative(frames,tip,base);
        error.head<3>()=target.topRightCorner<3,1>()-current.topRightCorner<3,1>();
        const Eigen::AngleAxisd angle(target.topLeftCorner<3,3>()*current.topLeftCorner<3,3>().transpose());
        error.tail<3>()=angle.axis()*angle.angle();
        if(error.head<3>().norm()<=position_tolerance&&(orientation_weight==0||error.tail<3>().norm()<=orientation_tolerance)) {reason=0;break;}
        if(iteration==max_iterations)break;
        if(elapsed(start)>=timeout_ms) {reason=2;break;}
        jacobian(m,frames,tip,base,active,n,j);
        j.bottomRows<3>()*=orientation_weight;
        weighted=error;weighted.tail<3>()*=orientation_weight;
        const Eigen::Matrix<double,6,6> normal=j.leftCols(n)*j.leftCols(n).transpose()+damping*damping*Eigen::Matrix<double,6,6>::Identity();
        delta.head(n)=j.leftCols(n).transpose()*normal.ldlt().solve(weighted);
        if(!delta.head(n).allFinite())return -1;
        const double factor=std::min(1.0,max_step/std::max(delta.head(n).cwiseAbs().maxCoeff(),1e-12));
        for(int i=0;i<n;++i) {
            const int index=active[i];const auto& joint=m.joints[index];
            result[index]=std::clamp(result[index]+factor*delta[i],joint.lower,joint.upper);
        }
    }
    stats[0]=error.head<3>().norm();stats[1]=error.tail<3>().norm();stats[2]=iteration;
    stats[3]=elapsed(start);stats[4]=reason;
    return reason==0?1:0;
}
