/* simplewalker state for state estimation 
March 2022 blame Niraj*/
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::MatrixXf, Eigen::Matrix3f, Eigen::VectorXf, Eigen::Vector3f;
typedef Eigen::VectorBlock<Eigen::VectorXf, 3> Block3f;

#define STATEIDX_POS 0
#define STATEIDX_EUL 3
#define STATEIDX_VEL 6
#define STATEIDX_ANGVEL 9

const int N = 12;

class RobotState {
    Matrix3f R_, RT_;
public:
    enum index {IDX_POS = 0, IDX_EUL = 3, IDX_VEL = 6, IDX_ANGVEL = 9};
    VectorXf vect;
    RobotState();
    void calculate(); //calculates the rotation matrix and leg pos after state values are set
    void calculate(Matrix3f *dRT_deul);
    //state elements
    const Vector3f pos() const;
    Block3f pos();
    Block3f euler();
    const Vector3f euler() const;
    Block3f vel();
    const Vector3f vel() const;
    Block3f angvel();
    const Vector3f angvel() const;
    float &alpha(), &beta(), &gamma();
    const float &alpha() const, &beta() const, &gamma() const;
    //calculated values
    const Matrix3f& R; //rotation matrix; world_frame = R * body_frame
    const Matrix3f& RT; //transposed rotation matrix; body_frame = RT * world_frame
    //Matrix3f * const &Jac_RT_Eul; // TODO: should this be part of state estimation calculated by calculate()
};

std::ostream &operator<<(std::ostream &output, const RobotState &State);
std::string state_CSV_header(void); //TODO list of const strings of names that get output by <<
