#include <gtest/gtest.h>

#include "tracker/matrix.hpp"

using tracker::Matrix;

TEST(Matrix, IdentityTimesMatrixIsUnchanged) {
    Matrix<2, 2> m;
    m(0, 0) = 1.0;
    m(0, 1) = 2.0;
    m(1, 0) = 3.0;
    m(1, 1) = 4.0;
    const auto result = Matrix<2, 2>::identity() * m;
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 2; ++c) {
            EXPECT_DOUBLE_EQ(result(r, c), m(r, c));
        }
    }
}

TEST(Matrix, TransposeSwapsDimensions) {
    Matrix<2, 3> m;
    m(0, 2) = 7.0;
    const Matrix<3, 2> t = m.transpose();
    EXPECT_DOUBLE_EQ(t(2, 0), 7.0);
}

TEST(Matrix, InverseOfInvertibleMatrix) {
    Matrix<2, 2> m;
    m(0, 0) = 4.0;
    m(0, 1) = 7.0;
    m(1, 0) = 2.0;
    m(1, 1) = 6.0;
    const auto inv = tracker::inverse(m);
    ASSERT_TRUE(inv.has_value());
    const auto product = m * *inv;
    EXPECT_NEAR(product(0, 0), 1.0, 1e-12);
    EXPECT_NEAR(product(0, 1), 0.0, 1e-12);
    EXPECT_NEAR(product(1, 0), 0.0, 1e-12);
    EXPECT_NEAR(product(1, 1), 1.0, 1e-12);
}

TEST(Matrix, InverseOfSingularMatrixReturnsNullopt) {
    Matrix<2, 2> m;
    m(0, 0) = 1.0;
    m(0, 1) = 2.0;
    m(1, 0) = 2.0;
    m(1, 1) = 4.0;
    EXPECT_FALSE(tracker::inverse(m).has_value());
}

// Compile-time check: the matrix math is usable in constant expressions.
static_assert((Matrix<3, 3>::identity() * Matrix<3, 3>::identity())(1, 1) == 1.0);
