#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;


void computeRotation(const Matrix3f& m){

  cout << "rotation matrix =\n" << m << endl<<endl;   
  AngleAxisf rotation_vector;
  rotation_vector.fromRotationMatrix(m);     
  cout << "rotation vector =\n angel = " << rotation_vector.angle()<<",   vector=" <<rotation_vector.axis().transpose() << endl;   
  Quaternionf q = Quaternionf(rotation_vector);
  cout << "Quaternionf  =\n" << q.coeffs().transpose() << endl<<endl;   

  Vector3f v(1, 2, 3);
  
  // 欧氏变换矩阵使用 Eigen::Isometry
  Isometry3f T = Isometry3f::Identity();                
  T.rotate(rotation_vector);                             // 按照rotation_vector进行旋转
  T.pretranslate(Vector3f(3, 2, 1));                     // 把平移向量设成(3,2,1)
  // 用变换矩阵进行坐标变换
  Vector3f v_transformed = T * v;                              // 相当于R*v+t
  cout << "v tranformed = \n" << v_transformed.transpose() << endl;

}
// 本程序演示了 Eigen 几何模块的使用方法

int main(int argc, char **argv) {
  //cout.precision(3);
  // 旋转坐标系
  cout<<endl<<"--------------rotating coordinate--------------------------"<<endl;
  Matrix3f rotating_m;
  rotating_m = AngleAxisf(10* (M_PI / 180), Vector3f::UnitZ())
    * AngleAxisf(20* (M_PI / 180), Vector3f::UnitY())
    * AngleAxisf(30* (M_PI / 180), Vector3f::UnitX());
  computeRotation(rotating_m);

  // 当前坐标系， 对应的旋转坐标系相反顺序
  cout<<endl<<"--------------fixed coordinate--------------------------"<<endl;
  Matrix3f fixed_m;
  fixed_m = AngleAxisf(30* (M_PI / 180), Vector3f::UnitX())
    * AngleAxisf(20* (M_PI / 180), Vector3f::UnitY())
    * AngleAxisf(10* (M_PI / 180), Vector3f::UnitZ());
  computeRotation(fixed_m);
  return 0;
}
