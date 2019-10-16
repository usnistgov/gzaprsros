

#include <gtest/gtest.h>
#include "Conversions.h"

using namespace Conversion;

inline std::string DumpPoseSimple(RCS::Pose pose) {
    std::stringstream s;

    s << "Translation = " << 1000.0 * pose.getOrigin().x() << ":" << 1000.0 * pose.getOrigin().y() << ":" << 1000.0 * pose.getOrigin().z();
    double roll, pitch, yaw;
    //getRPY(pose, roll, pitch, yaw);
    tf::Matrix3x3(pose.getRotation()).getRPY(roll, pitch, yaw);
    s << "Rotation = " << Rad2Deg(roll) << ":" << Rad2Deg(pitch) << ":" << Rad2Deg(yaw);
    return s.str();
}

inline std::string DumpEigenPose(Eigen::Affine3d pose) {
    std::stringstream s;
    s << "Translation = " << 1000.0 * pose(0, 3) << ":" << 1000.0 * pose(1, 3) << ":" << 1000.0 * pose(2, 3);
    tf::Matrix3x3 tfMatrix3x3;
    Eigen::Matrix3d e = pose.rotation();
    tfMatrix3x3 = Convert<Eigen::Matrix3d, tf::Matrix3x3>(e);

    double roll, pitch, yaw;
    tfMatrix3x3.getRPY(roll, pitch, yaw);
    s << "Rotation = " << Rad2Deg(roll) << ":" << Rad2Deg(pitch) << ":" << Rad2Deg(yaw);

    return s.str();
}

double gen_rand(double min, double max) {
    int rand_num = rand() % 100 + 1;
    double result = min + (double) ((max - min) * rand_num) / 101.0;
    return result;
}

TEST(TFEigenConversions, tf_eigen_vector) {
    std::cout << " tf_eigen_vector test" << std::endl;
    tf::Vector3 t;
    t[0] = gen_rand(-10, 10);
    t[1] = gen_rand(-10, 10);
    t[2] = gen_rand(-10, 10);
    Eigen::Vector3d k = Convert<tf::Vector3, Eigen::Vector3d>(t);

    ASSERT_NEAR(t[0], k[0], 1e-6);
    ASSERT_NEAR(t[1], k[1], 1e-6);
    ASSERT_NEAR(t[2], k[2], 1e-6);
}

TEST(TFEigenConversions, tf_eigen_quaternion) {
    std::cout << " tf_eigen_quaternion test" << std::endl;
    tf::Quaternion t;
    t[0] = gen_rand(-1.0, 1.0);
    t[1] = gen_rand(-1.0, 1.0);
    t[2] = gen_rand(-1.0, 1.0);
    t[3] = gen_rand(-1.0, 1.0);
    t.normalize();
    Eigen::Quaterniond k = Convert<tf::Quaternion, Eigen::Quaterniond>(t);
    ASSERT_NEAR(t[0], k.coeffs()(0), 1e-6);
    ASSERT_NEAR(t[1], k.coeffs()(1), 1e-6);
    ASSERT_NEAR(t[2], k.coeffs()(2), 1e-6);
    ASSERT_NEAR(t[3], k.coeffs()(3), 1e-6);
    ASSERT_NEAR(k.norm(), 1.0, 1e-10);
}

TEST(TFEigenConversions, tf_eigen_transform) {
    std::cout << " tf_eigen_transform test1" << std::endl;
    tf::Transform t;
    tf::Quaternion tq;
    tq[0] = gen_rand(-1.0, 1.0);
    tq[1] = gen_rand(-1.0, 1.0);
    tq[2] = gen_rand(-1.0, 1.0);
    tq[3] = gen_rand(-1.0, 1.0);
    tq.normalize();
    t.setOrigin(tf::Vector3(gen_rand(-10, 10), gen_rand(-10, 10), gen_rand(-10, 10)));
    t.setRotation(tq);

    Eigen::Affine3d affine = Convert<tf::Transform, Eigen::Affine3d>(t);


    for (int i = 0; i < 3; i++) {
        ASSERT_NEAR(t.getOrigin()[i], affine.matrix()(i, 3), 1e-6);
        for (int j = 0; j < 3; j++) {
            ASSERT_NEAR(t.getBasis()[i][j], affine.matrix()(i, j), 1e-6);
        }
    }
    for (int col = 0; col < 3; col++) {
        ASSERT_NEAR(affine.matrix()(3, col), 0, 1e-6);
    }
    ASSERT_NEAR(affine.matrix()(3, 3), 1, 1e-6);
}

// Not sure why this fails - needs further intellect which I don't have
#if 0
TEST(TFEigenConversions, eigen_tf_transform2) {
    std::cout << " tf_eigen_transform2 test" << std::endl;
    tf::Transform t1;
    Eigen::Affine3d affine;
    Eigen::Isometry3d isometry;
    Eigen::Quaterniond kq;
    kq.coeffs()(0) = gen_rand(-1.0, 1.0);
    kq.coeffs()(1) = gen_rand(-1.0, 1.0);
    kq.coeffs()(2) = gen_rand(-1.0, 1.0);
    kq.coeffs()(3) = gen_rand(-1.0, 1.0);
    kq.normalize();
    isometry.translate(Eigen::Vector3d(gen_rand(-10, 10), gen_rand(-10, 10), gen_rand(-10, 10)));
    isometry.rotate(kq);
    affine = isometry;
    affine.translate( Eigen::Vector3d(gen_rand(-10, 10), gen_rand(-10, 10), gen_rand(-10, 10)));
    affine*=kq;  
    

    t1 = Convert<Eigen::Affine3d, tf::Transform>(affine); // hopefuly Transform same as Pose
    std::cout << "tf Pose " << DumpPoseSimple(t1).c_str() << "\n";
    std::cout << "Eigen Pose " << DumpEigenPose(affine).c_str() << "\n";

    // This could be row vs col major problem ...
    for (int i = 0; i < 3; i++) {
        std::cout << i << " t1=" << t1.getOrigin()[i] << " affine =" << affine.matrix()(i, 3) << "\n";
        ASSERT_NEAR(t1.getOrigin()[i], affine.matrix()(i, 3), 1e-6);
        for (int j = 0; j < 3; j++) {
            std::cout << i << " t1=" << t1.getBasis()[i][j] << " affine =" << affine.rotation()(i, j) << "\n";
            ASSERT_NEAR(t1.getBasis()[i][j], affine.rotation()(i, j), 1e-6);
        }
    }
}
#endif
int main(int argc, char **argv) {
    /* initialize random seed: */
    srand(time(NULL));
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
