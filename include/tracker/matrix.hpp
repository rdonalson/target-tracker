#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace tracker {

/// Fixed-size, stack-allocated row-major matrix.
///
/// Sizes are compile-time constants, so dimension mismatches are compile
/// errors and no heap allocation ever occurs (a common requirement in
/// safety-critical / real-time code).
template <std::size_t Rows, std::size_t Cols>
class Matrix {
public:
    static constexpr std::size_t kRows = Rows;
    static constexpr std::size_t kCols = Cols;

    constexpr Matrix() = default;

    [[nodiscard]] static constexpr Matrix identity() noexcept
        requires(Rows == Cols)
    {
        Matrix m;
        for (std::size_t i = 0; i < Rows; ++i) {
            m(i, i) = 1.0;
        }
        return m;
    }

    [[nodiscard]] constexpr double& operator()(std::size_t r, std::size_t c) noexcept {
        return data_[r * Cols + c];
    }
    [[nodiscard]] constexpr double operator()(std::size_t r, std::size_t c) const noexcept {
        return data_[r * Cols + c];
    }

    [[nodiscard]] constexpr Matrix<Cols, Rows> transpose() const noexcept {
        Matrix<Cols, Rows> t;
        for (std::size_t r = 0; r < Rows; ++r) {
            for (std::size_t c = 0; c < Cols; ++c) {
                t(c, r) = (*this)(r, c);
            }
        }
        return t;
    }

    constexpr Matrix& operator+=(const Matrix& rhs) noexcept {
        for (std::size_t i = 0; i < Rows * Cols; ++i) {
            data_[i] += rhs.data_[i];
        }
        return *this;
    }
    constexpr Matrix& operator-=(const Matrix& rhs) noexcept {
        for (std::size_t i = 0; i < Rows * Cols; ++i) {
            data_[i] -= rhs.data_[i];
        }
        return *this;
    }

    friend constexpr Matrix operator+(Matrix lhs, const Matrix& rhs) noexcept { return lhs += rhs; }
    friend constexpr Matrix operator-(Matrix lhs, const Matrix& rhs) noexcept { return lhs -= rhs; }

private:
    std::array<double, Rows * Cols> data_{};
};

template <std::size_t R, std::size_t K, std::size_t C>
[[nodiscard]] constexpr Matrix<R, C> operator*(const Matrix<R, K>& a,
                                               const Matrix<K, C>& b) noexcept {
    Matrix<R, C> out;
    for (std::size_t r = 0; r < R; ++r) {
        for (std::size_t c = 0; c < C; ++c) {
            double sum = 0.0;
            for (std::size_t k = 0; k < K; ++k) {
                sum += a(r, k) * b(k, c);
            }
            out(r, c) = sum;
        }
    }
    return out;
}

template <std::size_t N>
using Vector = Matrix<N, 1>;

/// Inverse of a 2x2 matrix. Returns std::nullopt if the matrix is
/// (numerically) singular instead of throwing or dividing by zero.
[[nodiscard]] inline std::optional<Matrix<2, 2>> inverse(const Matrix<2, 2>& m) noexcept {
    constexpr double kEpsilon = 1e-12;
    const double det = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
    if (std::abs(det) < kEpsilon) {
        return std::nullopt;
    }
    Matrix<2, 2> inv;
    inv(0, 0) = m(1, 1) / det;
    inv(0, 1) = -m(0, 1) / det;
    inv(1, 0) = -m(1, 0) / det;
    inv(1, 1) = m(0, 0) / det;
    return inv;
}

}  // namespace tracker
