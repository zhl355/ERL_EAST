#pragma once

#include <vector>

#include <Eigen/Core>

#include <erl_utilities/trajectories/random_polynomial_matrix.h>
#include <erl_utilities/trajectories/piecewise_polynomial.h>

//namespace drake {
namespace trajectories {
//namespace test {

/**
 * Obtains a random PiecewisePolynomial with the given @p segment_times.  Each
 * segment will have a matrix of random Polynomials of the specified size.
 */
template<typename CoefficientType = double>
PiecewisePolynomial<CoefficientType>
MakeRandomPiecewisePolynomial(Eigen::Index rows, Eigen::Index cols,
                              Eigen::Index num_coefficients_per_polynomial,
                              const std::vector<double> &segment_times) {
  Eigen::Index num_segments = static_cast<Eigen::Index>(segment_times.size() - 1);
  typedef Polynomial<CoefficientType> PolynomialType;
  typedef Eigen::Matrix<PolynomialType, Eigen::Dynamic, Eigen::Dynamic> PolynomialMatrix;
  std::vector<PolynomialMatrix> polynomials;
  for (Eigen::Index segment_index = 0; segment_index < num_segments;
       ++segment_index) {
    polynomials.push_back(RandomPolynomialMatrix<CoefficientType>(num_coefficients_per_polynomial, rows, cols));
  }
  return PiecewisePolynomial<CoefficientType>(polynomials, segment_times);
}

//}  // namespace test
}  // namespace trajectories
//}  // namespace drake
