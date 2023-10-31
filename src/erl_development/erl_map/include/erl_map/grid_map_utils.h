#ifndef __GRID_MAP_UTILS_H_
#define __GRID_MAP_UTILS_H_
#include <cmath>
#include <vector>
#include <array>
#include <limits>
#include <Eigen/Core>
#include <iostream> // for debugging

namespace erl {

/**
 * @file grid_map_utils.h
 * @brief Mapping utility functions, useful for discretizing n-dimensional maps, and converting metric to cell coordinates.
 */

inline int meters2cells(double datam, double min, double res) {
  return static_cast<int>(std::floor((datam - min) / res));
}

inline double cells2meters(int datac, double min, double res) { return (static_cast<double>(datac) + 0.5) * res + min; }

inline double meters2cells_cont(double datam, double dim_min, double res) { return (datam - dim_min) / res - 0.5; }

// returns the first odd integer larger than x
inline int odd_ceil(double x) {
  if (std::abs(std::floor(x) - x) <= std::numeric_limits<double>::epsilon())
    x = std::floor(x);
  int ocx = static_cast<int>(std::ceil(x));
  return (ocx % 2 == 0) ? (ocx + 1) : (ocx);
}


/******************************************************************************/
// Array Implementations  
/******************************************************************************/
template<size_t D>
inline std::array<int, D> meters2cells(std::array<double, D> const &datam,
                                       std::array<double, D> const &dim_min,
                                       std::array<double, D> const &res) {
  std::array<int, D> datac;
  for (unsigned k = 0; k < D; ++k)
    datac[k] = meters2cells(datam[k], dim_min[k], res[k]);
  return datac;
}

template<size_t D>
inline std::array<double, D> cells2meters(std::array<int, D> const &datac,
                                          std::array<double, D> const &dim_min,
                                          std::array<double, D> const &res) {
  std::array<double, D> datam;
  for (unsigned k = 0; k < D; ++k)
    datam[k] = cells2meters(datac[k], dim_min[k], res[k]);
  return datam;
}

template<size_t D>
inline std::array<double, D> meters2cells_cont(std::array<double, D> const &datam,
                                               std::array<double, D> const &dim_min,
                                               std::array<double, D> const &res) {
  std::array<double, D> datac;
  for (unsigned k = 0; k < D; ++k)
    datac[k] = meters2cells_cont(datam[k], dim_min[k], res[k]);
  return datac;
}

// Row major order as in C++
inline std::size_t subv2ind_rowmajor(int *datac_begin, int *datac_end, int *size_begin, int *size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *(datac_end - 1);
  std::size_t prod = 1;
  int *it1 = datac_end - 2;
  int *it2 = size_end - 1;
  for (; it1 != (datac_begin - 1) && it2 != size_begin; --it1, --it2) {
    prod *= (*it2);
    idx += prod * (*it1);
  }
  return idx;
}

inline std::size_t subv2ind_rowmajor(int *datac_begin, int *datac_end,
                                     std::vector<int>::const_iterator const &size_begin,
                                     std::vector<int>::const_iterator const &size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *(datac_end - 1);
  std::size_t prod = 1;
  int *it1 = datac_end - 2;
  std::vector<int>::const_iterator it2 = size_end - 1;
  for (; it1 != (datac_begin - 1) && it2 != size_begin; --it1, --it2) {
    prod *= (*it2);
    idx += prod * (*it1);
  }
  return idx;
}

template<std::size_t D>
inline std::array<int, D> ind2subv_rowmajor(std::size_t ind, std::array<int, D> const &sz) {
  std::array<int, D> subv;
  for (int k = D - 1; k >= 0; --k) {
    subv[k] = ind % sz[k];
    ind -= subv[k];
    ind /= sz[k];
  }
  return subv;
}

// Column major order as in MATLAB
inline std::size_t subv2ind_colmajor(int *datac_begin, int *datac_end, int *size_begin, int *size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *datac_begin;
  std::size_t prod = 1;
  int *it1 = datac_begin + 1;
  int *it2 = size_begin;
  for (; it1 != datac_end && it2 != (size_end - 1); ++it1, ++it2) {
    prod *= (*it2);
    idx += (*it1) * prod;
  }
  return idx;
}

inline std::size_t subv2ind_colmajor(int *datac_begin, int *datac_end,
                                     std::vector<int>::const_iterator const &size_begin,
                                     std::vector<int>::const_iterator const &size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *datac_begin;
  std::size_t prod = 1;
  int *it1 = datac_begin + 1;
  std::vector<int>::const_iterator it2 = size_begin;
  for (; it1 != datac_end && it2 != (size_end - 1); ++it1, ++it2) {
    prod *= (*it2);
    idx += (*it1) * prod;
  }
  return idx;
}

template<std::size_t D>
inline std::array<int, D> ind2subv_colmajor(std::size_t ind, std::array<int, D> const &sz) {
  std::array<int, D> subv;
  for (size_t k = 0; k < D; ++k) {
    subv[k] = ind % sz[k];
    ind -= subv[k];
    ind /= sz[k];
  }
  return subv;
}



/******************************************************************************/
// Vector Implementations
/******************************************************************************/
inline std::vector<int> meters2cells(std::vector<double>::const_iterator const &datam_begin,
                                     std::vector<double>::const_iterator const &datam_end,
                                     std::vector<double>::const_iterator const &dim_min_begin,
                                     std::vector<double>::const_iterator const &dim_min_end,
                                     std::vector<double>::const_iterator const &res_begin,
                                     std::vector<double>::const_iterator const &res_end) {
  std::vector<int> datac(std::distance(datam_begin, datam_end));
  std::vector<double>::const_iterator it1 = datam_begin;
  std::vector<double>::const_iterator it2 = dim_min_begin;
  std::vector<double>::const_iterator it3 = res_begin;
  for (unsigned k = 0; it1 != datam_end; ++it1, ++it2, ++it3, ++k)
    datac[k] = meters2cells(*it1, *it2, *it3);
  return datac;
}

inline std::vector<double> cells2meters(std::vector<int>::const_iterator const &datac_begin,
                                        std::vector<int>::const_iterator const &datac_end,
                                        std::vector<double>::const_iterator const &dim_min_begin,
                                        std::vector<double>::const_iterator const &dim_min_end,
                                        std::vector<double>::const_iterator const &res_begin,
                                        std::vector<double>::const_iterator const &res_end) {
  std::vector<double> datam(std::distance(datac_begin, datac_end));
  std::vector<int>::const_iterator it1 = datac_begin;
  std::vector<double>::const_iterator it2 = dim_min_begin;
  std::vector<double>::const_iterator it3 = res_begin;
  for (unsigned k = 0; it1 != datac_end; ++it1, ++it2, ++it3, ++k)
    datam[k] = meters2cells(*it1, *it2, *it3);
  return datam;
}

inline std::vector<double> meters2cells_cont(
    std::vector<double>::const_iterator const &datam_begin,
    std::vector<double>::const_iterator const &datam_end,
    std::vector<double>::const_iterator const &dim_min_begin,
    std::vector<double>::const_iterator const &dim_min_end,
    std::vector<double>::const_iterator const &res_begin,
    std::vector<double>::const_iterator const &res_end) {
  std::vector<double> datac(std::distance(datam_begin, datam_end));
  std::vector<double>::const_iterator it1 = datam_begin;
  std::vector<double>::const_iterator it2 = dim_min_begin;
  std::vector<double>::const_iterator it3 = res_begin;
  for (unsigned k = 0; it1 != datam_end; ++it1, ++it2, ++it3, ++k)
    datac[k] = meters2cells_cont(*it1, *it2, *it3);
  return datac;
}

inline std::vector<int> meters2cells(std::vector<double> const &datam,
                                     std::vector<double> const &dim_min,
                                     std::vector<double> const &res) {
  std::vector<int> datac(datam.size());
  for (unsigned k = 0; k < datam.size(); ++k)
    datac[k] = meters2cells(datam[k], dim_min[k], res[k]);
  return datac;
}

inline std::vector<int> meters2cells(std::vector<double> const &datam,
                                     std::vector<double> const &dim_min, double res) {
  std::vector<int> datac(datam.size());
  for (unsigned k = 0; k < datam.size(); ++k)
    datac[k] = meters2cells(datam[k], dim_min[k], res);
  return datac;
}

inline std::vector<double> cells2meters(std::vector<int> const &datac,
                                        std::vector<double> const &dim_min, std::vector<double> const &res) {
  std::vector<double> datam(datac.size());
  for (unsigned k = 0; k < datac.size(); ++k)
    datam[k] = cells2meters(datac[k], dim_min[k], res[k]);
  return datam;
}

inline std::vector<double> cells2meters(std::vector<int> const &datac,
                                        std::vector<double> const &dim_min, double res) {
  std::vector<double> datam(datac.size());
  for (unsigned k = 0; k < datac.size(); ++k)
    datam[k] = cells2meters(datac[k], dim_min[k], res);
  return datam;
}

inline std::vector<double> meters2cells_cont(const std::vector<double> &datam,
                                             const std::vector<double> &dim_min,
                                             const std::vector<double> &res) {
  std::vector<double> datac(datam.size());
  for (unsigned k = 0; k < datam.size(); ++k)
    datac[k] = meters2cells_cont(datam[k], dim_min[k], res[k]);
  return datac;
}

inline std::vector<double> meters2cells_cont(const std::vector<double> &datam,
                                             const std::vector<double> &dim_min,
                                             double res) {
  std::vector<double> datac(datam.size());
  for (unsigned k = 0; k < datam.size(); ++k)
    datac[k] = meters2cells_cont(datam[k], dim_min[k], res);
  return datac;
}

// Row major order as in C++
inline std::size_t subv2ind_rowmajor(std::vector<int>::const_iterator const &datac_begin,
                                     std::vector<int>::const_iterator const &datac_end,
                                     std::vector<int>::const_iterator const &size_begin,
                                     std::vector<int>::const_iterator const &size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *(datac_end - 1);
  std::size_t prod = 1;
  std::vector<int>::const_iterator it1 = datac_end - 2;
  std::vector<int>::const_iterator it2 = size_end - 1;
  for (; it1 != (datac_begin - 1) && it2 != size_begin; --it1, --it2) {
    prod *= (*it2);
    idx += prod * (*it1);
  }
  return idx;
}

inline std::vector<int> ind2subv_rowmajor(std::size_t ind,
                                          const std::vector<int>::const_iterator &size_begin,
                                          const std::vector<int>::const_iterator &size_end) {
  const std::size_t ndims = std::distance(size_begin, size_end);
  std::vector<int> subv(ndims);
  std::vector<int>::const_iterator it = size_end - 1;
  for (int k = ndims - 1; k >= 0; --k, --it) {
    subv[k] = ind % (*it);
    ind -= subv[k];
    ind /= (*it);
  }
  return subv;
}

// Column major order as in MATLAB
inline std::size_t subv2ind_colmajor(std::vector<int>::const_iterator const &datac_begin,
                                     std::vector<int>::const_iterator const &datac_end,
                                     std::vector<int>::const_iterator const &size_begin,
                                     std::vector<int>::const_iterator const &size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *datac_begin;
  std::size_t prod = 1;
  std::vector<int>::const_iterator it1 = datac_begin + 1;
  std::vector<int>::const_iterator it2 = size_begin;
  for (; it1 != datac_end && it2 != (size_end - 1); ++it1, ++it2) {
    prod *= (*it2);
    idx += (*it1) * prod;
  }
  return idx;
}

inline std::vector<int> ind2subv_colmajor(std::size_t ind,
                                          const std::vector<int>::const_iterator &size_begin,
                                          const std::vector<int>::const_iterator &size_end) {
  const std::size_t ndims = std::distance(size_begin, size_end);
  std::vector<int> subv(ndims);
  std::vector<int>::const_iterator it = size_begin;
  for (std::size_t k = 0; k < ndims; ++k, ++it) {
    subv[k] = ind % (*it);
    ind -= subv[k];
    ind /= (*it);
  }
  return subv;
}


/******************************************************************************/
// Matrix Implementations
/******************************************************************************/
inline Eigen::MatrixXi meters2cells(const Eigen::MatrixXd &datam,
                                    const Eigen::VectorXd &dim_min,
                                    const Eigen::VectorXd &res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXi datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells(datam(d, p), dim_min(d), res(d));
  return datac;
}

inline Eigen::MatrixXi meters2cells(const Eigen::MatrixXd &datam,
                                    const Eigen::VectorXd &dim_min,
                                    double res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXi datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells(datam(d, p), dim_min(d), res);
  return datac;
}

inline Eigen::MatrixXd cells2meters(const Eigen::MatrixXi &datac,
                                    const Eigen::VectorXd &dim_min,
                                    const Eigen::VectorXd &res) {
  // datac = [num_dim x num_pts]
  const size_t num_dim = datac.rows();
  const size_t num_pts = datac.cols();
  Eigen::MatrixXd datam(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datam(d, p) = cells2meters(datac(d, p), dim_min(d), res(d));
  return datam;
}

inline Eigen::MatrixXd cells2meters(const Eigen::MatrixXi &datac,
                                    const Eigen::VectorXd &dim_min,
                                    double res) {
  // datac = [num_dim x num_pts]
  const size_t num_dim = datac.rows();
  const size_t num_pts = datac.cols();
  Eigen::MatrixXd datam(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datam(d, p) = cells2meters(datac(d, p), dim_min(d), res);
  return datam;
}

inline Eigen::MatrixXd meters2cells_cont(const Eigen::MatrixXd &datam,
                                         const Eigen::VectorXd &dim_min,
                                         const Eigen::VectorXd &res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXd datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells_cont(datam(d, p), dim_min(d), res(d));
  return datac;
}

inline Eigen::MatrixXd meters2cells_cont(const Eigen::MatrixXd &datam,
                                         const Eigen::VectorXd &dim_min,
                                         double res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXd datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells_cont(datam(d, p), dim_min(d), res);
  return datac;
}

inline Eigen::MatrixXi meters2cells(const Eigen::MatrixXd &datam,
                                    const std::vector<double> &dim_min,
                                    const std::vector<double> &res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXi datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells(datam(d, p), dim_min[d], res[d]);
  return datac;
}

inline Eigen::MatrixXi meters2cells(const Eigen::MatrixXd &datam,
                                    const std::vector<double> &dim_min,
                                    double res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXi datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells(datam(d, p), dim_min[d], res);
  return datac;
}

inline Eigen::MatrixXd cells2meters(const Eigen::MatrixXi &datac,
                                    const std::vector<double> &dim_min,
                                    const std::vector<double> &res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datac.rows();
  const size_t num_pts = datac.cols();
  Eigen::MatrixXd datam(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datam(d, p) = cells2meters(datac(d, p), dim_min[d], res[d]);
  return datam;
}

inline Eigen::MatrixXd cells2meters(const Eigen::MatrixXi &datac,
                                    const std::vector<double> &dim_min,
                                    double res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datac.rows();
  const size_t num_pts = datac.cols();
  Eigen::MatrixXd datam(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datam(d, p) = cells2meters(datac(d, p), dim_min[d], res);
  return datam;
}

inline Eigen::MatrixXd meters2cells_cont(const Eigen::MatrixXd &datam,
                                         const std::vector<double> &dim_min,
                                         const std::vector<double> &res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXd datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells_cont(datam(d, p), dim_min[d], res[d]);
  return datac;
}

inline Eigen::MatrixXd meters2cells_cont(const Eigen::MatrixXd &datam,
                                         const std::vector<double> &dim_min,
                                         double res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXd datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells_cont(datam(d, p), dim_min[d], res);
  return datac;
}
/******************************************************************************/


/******************************************************************************/
// Algorithms
/******************************************************************************/


template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>
inflate_map2d(const Eigen::MatrixBase<Derived> &cmap,
              const std::vector<double> &res, double rad) {
  Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> omap = cmap;
  if (rad < 0.00001)
    return omap; // nothing to do here

  int bbx = static_cast<int>(std::ceil(rad / res[0]));
  int bby = static_cast<int>(std::ceil(rad / res[1]));

  std::vector<std::array<int, 2>> ns;
  ns.reserve((2 * bbx + 1) * (2 * bby + 1));
  for (int nx = -bbx; nx <= bbx; ++nx)
    for (int ny = -bby; ny <= bby; ++ny) {
      double d = std::hypot(nx * res[0], ny * res[1]);
      if (0 < d && d <= rad)
        ns.push_back({nx, ny});
    }

  for (unsigned int x = 0; x < cmap.rows(); ++x)
    for (unsigned int y = 0; y < cmap.cols(); ++y) {
      if (cmap(x, y) > 0) {
        for (const auto &it : ns) {
          int nx = x + it[0], ny = y + it[1];
          if (0 <= nx && nx < cmap.rows() && 0 <= ny && ny < cmap.cols())
            omap(nx, ny) = cmap(x, y);
        }
      }
    }
  return omap;
}

template<class T>
inline std::vector<T>
inflateMap(const std::vector<T> &map_cost,
           const std::vector<int> &map_size,
           const std::vector<double> &map_resolution,
           bool is_rowmajor,
           const std::vector<double> &inflation_radius) {

  // find the number of cells to inflate per dimension
  std::vector<int> extents(map_resolution.size());
  size_t num_ns = 1;
  for (size_t i = 0; i < map_resolution.size(); ++i) {
    extents[i] = 2 * static_cast<int>(std::ceil(inflation_radius[i] / map_resolution[i])) + 1;
    num_ns *= extents[i];
  }

  // compute the neighbor coordinates
  std::vector<std::vector<int>> ns;
  ns.reserve(num_ns);
  for (size_t i = 0; i < num_ns; ++i) {
    // convert the linear index of every potential neighbor
    // to an index in [-(extents[i]-1)/2,(extents[i]-1)/2]
    // and check if this falls within the axis aligned ellipsoid
    // defined by inflation_radius
    std::vector<int> subv = ind2subv_colmajor(i, extents.begin(), extents.end());
    //std::cout << subv << std::endl;
    double d = 0.0; // ellipsoid radius
    for (size_t k = 0; k < map_size.size(); ++k) {
      subv[k] -= (extents[k] - 1) / 2; // add offset (i.e., cells2meters)
      double tmp = subv[k] * (map_resolution[k] / inflation_radius[k]);
      d += tmp * tmp;
    }

    if (0.0 < d && d <= 1.000001)
      ns.push_back(subv);
  }

  std::vector<T> inflated_cost(map_cost);
  for (size_t i = 0; i < map_cost.size(); ++i)
    if (map_cost[i] > T(0)) {
      // get coordinates of ith entry
      std::vector<int> subv = is_rowmajor ? ind2subv_rowmajor(i, map_size.begin(), map_size.end()) :
                              ind2subv_colmajor(i, map_size.begin(), map_size.end());
      // find the neighbors and make them occupied
      for (const auto &it : ns) {
        bool valid = true;
        std::vector<int> neib_coord(it);
        for (size_t k = 0; k < map_size.size(); ++k) {
          neib_coord[k] += subv[k];
          if (neib_coord[k] < 0 || neib_coord[k] >= map_size[k]) {
            valid = false;
            break;
          }
        }
        if (valid)
          inflated_cost[is_rowmajor ? subv2ind_rowmajor(neib_coord.begin(),
                                                        neib_coord.end(),
                                                        map_size.begin(),
                                                        map_size.end()) :
                        subv2ind_colmajor(neib_coord.begin(), neib_coord.end(), map_size.begin(), map_size.end())] =
              map_cost[i];
      }
    }
  return inflated_cost;
}


}
#endif
