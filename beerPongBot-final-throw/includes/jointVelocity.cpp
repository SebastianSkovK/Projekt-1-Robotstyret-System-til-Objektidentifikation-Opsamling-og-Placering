
#include "jointVelocity.h"

cv::Matx31d cross (cv::Matx31d a, cv::Matx31d b)
{
    return cv::Matx31d (
        a(1) * b(2) - a(2) * b(1),
        a(2) * b(0) - a(0) * b(2),
        a(0) * b(1) - a(1) * b(0));
}

cv::Matx61d getJointVelocities (cv::Matx61d q, cv::Matx61d endEffectorVelocity)
{
    cv::Matx61d sinQ = cv::Matx61d::zeros ();

    for (int i = 0; i < 6; i++)
    {
        sinQ (i) = std::sin (q (i));
    }

    cv::Matx61d cosQ = cv::Matx61d::zeros ();

    for (int i = 0; i < 6; i++)
    {
        cosQ (i) = std::cos (q (i));
    }

    cv::Matx44d A1 = cv::Matx44d(
        cosQ(0), 0,  sinQ(0), 0,
        sinQ(0), 0, -cosQ(0), 0,
          0, 1,    0, 0.08916,
          0, 0,    0,       1);

    cv::Matx44d A2 = cv::Matx44d(
        cosQ(1), -sinQ(1), 0, -0.425 * cosQ(1),
        sinQ(1),  cosQ(1), 0, -0.425 * sinQ(1),
          0,    0, 1, 0,
          0,    0, 0, 1);

    // Define A3
    cv::Matx44d A3 = cv::Matx44d(
        cosQ(2), -sinQ(2), 0, -0.392 * cosQ(2),
        sinQ(2), cosQ(2), 0, -0.392 * sinQ(2),
        0, 0, 1, 0,
        0, 0, 0, 1);

    // Define A4
    cv::Matx44d A4 = cv::Matx44d(
        cosQ(3), 0, sinQ(3), 0,
        sinQ(3), 0, -cosQ(3), 0,
        0, 1, 0, 0.1092,
        0, 0, 0, 1);

    // Define A5
    cv::Matx44d A5 = cv::Matx44d(
        cosQ(4), 0, -sinQ(4), 0,
        sinQ(4), 0, cosQ(4), 0,
        0, -1, 0, 0.0947,
        0, 0, 0, 1);

    double tcpOffset = 0.17;
    // Define A6
    cv::Matx44d A6 = cv::Matx44d(
        cosQ(5), 0, -sinQ(5), 0,
        sinQ(5), 0, cosQ(5), 0,
        0, 0, 1, 0.0823 + tcpOffset,
        0, 0, 0, 1);

    std::vector<cv::Matx44d> T;

    T.push_back (A1);
    T.push_back (T[0] * A2);
    T.push_back (T[1] * A3);
    T.push_back (T[2] * A4);
    T.push_back (T[3] * A5);
    T.push_back (T[4] * A6);

    std::vector<cv::Matx31d> P = { cv::Matx31d::zeros () };

    for (int i = 0; i < 6; i++)
    {
        P.push_back (cv::Matx31d (T[i](0, 3), T[i](1, 3), T[i](2, 3)));
    }

    std::vector<cv::Matx31d> Z = { cv::Matx31d (0, 0, 1) };

    for (int i = 0; i < 5; i++)
    {
        Z.push_back (cv::Matx31d (T[i](0, 2), T[i](1, 2), T[i](2, 2)));
    }

    cv::Matx66d J = cv::Matx66d::zeros ();

    for (int c = 0; c < 6; c++)
    {

        cv::Matx31d jp = cross (Z[c], P[6] - P[c]);

        for (int r = 0; r < 3; r++)
        {
            J (r, c) = jp(r);
        }

        cv::Matx31d jo = Z[c];

        for (int r = 0; r < 3; r++)
        {
            J (r + 3, c) = jo(r);
        }
    }

    cv::Matx66d Ji = J.inv ();

    return Ji * endEffectorVelocity;
}
cv::Matx31d getJointVelocitiesSEW (cv::Matx61d q, cv::Matx61d endEffectorVelocity)
{
    cv::Matx61d sinQ = cv::Matx61d::zeros ();

    for (int i = 0; i < 6; i++)
    {
        sinQ (i) = std::sin (q (i));
    }

    cv::Matx61d cosQ = cv::Matx61d::zeros ();

    for (int i = 0; i < 6; i++)
    {
        cosQ (i) = std::cos (q (i));
    }

    cv::Matx44d A1 = cv::Matx44d(
        cosQ(0), 0,  sinQ(0), 0,
        sinQ(0), 0, -cosQ(0), 0,
          0, 1,    0, 0.08916,
          0, 0,    0,       1);

    cv::Matx44d A2 = cv::Matx44d(
        cosQ(1), -sinQ(1), 0, -0.425 * cosQ(1),
        sinQ(1),  cosQ(1), 0, -0.425 * sinQ(1),
          0,    0, 1, 0,
          0,    0, 0, 1);

    // Define A3
    cv::Matx44d A3 = cv::Matx44d(
        cosQ(2), -sinQ(2), 0, -0.392 * cosQ(2),
        sinQ(2), cosQ(2), 0, -0.392 * sinQ(2),
        0, 0, 1, 0,
        0, 0, 0, 1);

    // Define A4
    cv::Matx44d A4 = cv::Matx44d(
        cosQ(3), 0, sinQ(3), 0,
        sinQ(3), 0, -cosQ(3), 0,
        0, 1, 0, 0.1092,
        0, 0, 0, 1);

    // Define A5
    cv::Matx44d A5 = cv::Matx44d(
        cosQ(4), 0, -sinQ(4), 0,
        sinQ(4), 0, cosQ(4), 0,
        0, -1, 0, 0.0947,
        0, 0, 0, 1);

    double tcpOffset = 0.17;
    // Define A6
    cv::Matx44d A6 = cv::Matx44d(
        cosQ(5), 0, -sinQ(5), 0,
        sinQ(5), 0, cosQ(5), 0,
        0, 0, 1, 0.0823 + tcpOffset,
        0, 0, 0, 1);

    std::vector<cv::Matx44d> T;

    T.push_back (A1);
    T.push_back (T[0] * A2);
    T.push_back (T[1] * A3);
    T.push_back (T[2] * A4);
    T.push_back (T[3] * A5);
    T.push_back (T[4] * A6);

    std::vector<cv::Matx31d> P = { cv::Matx31d::zeros () };

    for (int i = 0; i < 6; i++)
    {
        P.push_back (cv::Matx31d (T[i](0, 3), T[i](1, 3), T[i](2, 3)));
    }

    std::vector<cv::Matx31d> Z = { cv::Matx31d (0, 0, 1) };

    for (int i = 0; i < 5; i++)
    {
        Z.push_back (cv::Matx31d (T[i](0, 2), T[i](1, 2), T[i](2, 2)));
    }

    cv::Matx<double, 6, 3> J = cv::Matx<double, 6, 3>::zeros ();

    for (int c = 1; c <= 3; c++)
    {

        cv::Matx31d jp = cross (Z[c], P[6] - P[c]);

        for (int r = 0; r < 3; r++)
        {
            J (r, c - 1) = jp(r);
        }

        cv::Matx31d jo = Z[c];

        for (int r = 0; r < 3; r++)
        {
            J (r + 3, c - 1) = jo(r);
        }
    }

    cv::Matx<double, 3, 6> Ji = (J.t () * J).inv() * J.t();

    return Ji * endEffectorVelocity;
}

cv::Matx31d getTcpPosToGivenQ (cv::Matx61d q)
{
    cv::Matx61d sinQ = cv::Matx61d::zeros ();

    for (int i = 0; i < 6; i++)
    {
        sinQ (i) = std::sin (q (i));
    }

    cv::Matx61d cosQ = cv::Matx61d::zeros ();

    for (int i = 0; i < 6; i++)
    {
        cosQ (i) = std::cos (q (i));
    }

    cv::Matx44d A1 = cv::Matx44d(
        cosQ(0), 0,  sinQ(0), 0,
        sinQ(0), 0, -cosQ(0), 0,
          0, 1,    0, 0.08916,
          0, 0,    0,       1);

    cv::Matx44d A2 = cv::Matx44d(
        cosQ(1), -sinQ(1), 0, -0.425 * cosQ(1),
        sinQ(1),  cosQ(1), 0, -0.425 * sinQ(1),
          0,    0, 1, 0,
          0,    0, 0, 1);

    // Define A3
    cv::Matx44d A3 = cv::Matx44d(
        cosQ(2), -sinQ(2), 0, -0.392 * cosQ(2),
        sinQ(2), cosQ(2), 0, -0.392 * sinQ(2),
        0, 0, 1, 0,
        0, 0, 0, 1);

    // Define A4
    cv::Matx44d A4 = cv::Matx44d(
        cosQ(3), 0, sinQ(3), 0,
        sinQ(3), 0, -cosQ(3), 0,
        0, 1, 0, 0.1092,
        0, 0, 0, 1);

    // Define A5
    cv::Matx44d A5 = cv::Matx44d(
        cosQ(4), 0, -sinQ(4), 0,
        sinQ(4), 0, cosQ(4), 0,
        0, -1, 0, 0.0947,
        0, 0, 0, 1);

    // Define A6
    double tcpOffset = 0.17;
    cv::Matx44d A6 = cv::Matx44d(
        cosQ(5), 0, -sinQ(5), 0,
        sinQ(5), 0, cosQ(5), 0,
        0, 0, 1, (0.0823+tcpOffset),
        0, 0, 0, 1);

    cv::Matx44d T = A1*A2*A3*A4*A5*A6;

    return cv::Matx31d(T(0,3),T(1,3),T(2,3));
}

cv::Matx61d vecToMat (std::vector<double> vec)
{
    cv::Matx61d mat;
    for(int i = 0; i < vec.size(); ++i){
        mat(i) = vec[i];
    }
    return mat;
}

std::vector<double> matToVec (cv::Matx61d mat)
{
    std::vector<double> vec;
    for(int i = 0; i < 6; ++i){
        vec.push_back(mat(i));
    }
    return vec;
}

cv::Matx41d d3Tod4 (cv::Matx31d m){
    return cv::Matx41d(m(0), m(1), m(2), 1);
}

cv::Matx31d d4Tod3 (cv::Matx41d m){
    return cv::Matx31d(m(0), m(1), m(2));
}