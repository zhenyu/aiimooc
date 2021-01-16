#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;


void computeRotation(const Matrix3f& m){

  cout << "rotation matrix =\n" << m << endl;   
  
  AngleAxisf rotation_vector(m);     
  
  Quaternionf q = Quaternionf(rotation_vector);
  
  // 用 AngleAxis 可以进行坐标变换
  Vector3f v(1, 2, 3);
  Vector3f v_rotated = rotation_vector * v;
  
  // 欧氏变换矩阵使用 Eigen::Isometry
  Isometry3f T = Isometry3f::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
  T.rotate(rotation_vector);                                     // 按照rotation_vector进行旋转
  T.pretranslate(Vector3f(3, 2, 1));                     // 把平移向量设成(1,3,4)
  cout << "Transform matrix = \n" << T.matrix() << endl;
  
}
// 本程序演示了 Eigen 几何模块的使用方法

int main(int argc, char **argv) {
  cout.precision(3);
  // 旋转坐标系
  Matrix3f rotating_m;
  rotating_m = AngleAxisf(10* (M_PI / 180), Vector3f::UnitZ())
    * AngleAxisf(20* (M_PI / 180), Vector3f::UnitY())
    * AngleAxisf(30* (M_PI / 180), Vector3f::UnitX());
  computeRotation(rotating_m);

  // 当前坐标系， 对应的旋转坐标系相反顺序
  Matrix3f fixed_m;
  fixed_m = AngleAxisf(30* (M_PI / 180), Vector3f::UnitX())
    * AngleAxisf(20* (M_PI / 180), Vector3f::UnitY())
    * AngleAxisf(10* (M_PI / 180), Vector3f::UnitZ());
  computeRotation(fixed_m);
  return 0;
}
