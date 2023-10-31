#ifndef __GRID_MAP_H_
#define __GRID_MAP_H_

#include <vector>

#include "erl_map/grid_map_utils.h"


namespace erl {

template <typename T>
class GridMap {
  std::vector<double> min_;
  std::vector<double> max_;
  std::vector<double> res_;
  std::vector<int> size_;
  std::vector<double> origin_;
  std::vector<int> origincells_;
  bool row_major_{false};  // Indicates that the map is stored in row major order. Default to column-major.
  std::vector<T> map_;

 public:
  GridMap() {}  // default constructor

  GridMap(const std::vector<double> &mapmin, const std::vector<double> &mapmax, const std::vector<double> &mapres,
          bool rowmajor = false)
      : GridMap(mapmin.size(), mapmin.data(), mapmax.data(), mapres.data(), rowmajor) {}

  GridMap(const std::vector<int> &mapdim, const std::vector<double> &origin, const std::vector<double> &mapres,
          bool rowmajor = false)
      : GridMap(mapdim.size(), mapdim.data(), origin.data(), mapres.data(), rowmajor) {}

  GridMap(unsigned numdim, const double *mapmin, const double *mapmax, const double *mapres, bool rowmajor = false) {
    initMapData1(numdim, mapmin, mapmax, mapres, rowmajor);
  }

  GridMap(unsigned numdim, const int *mapdim, const double *origin, const double *mapres, bool rowmajor = false) {
    initMapData2(numdim, mapdim, origin, mapres, rowmajor);
  }

  std::vector<double> const &min() const { return min_; }
  std::vector<double> const &max() const { return max_; }
  std::vector<double> const &res() const { return res_; }
  std::vector<int> const &size() const { return size_; }
  std::vector<double> const &origin() const { return origin_; }
  std::vector<int> const &origincells() const { return origincells_; }
  std::vector<T> const &map() const { return map_; }
  size_t dim() const { return size_.size(); }
  bool rowmajor() const { return row_major_; }

  std::size_t totalSize() const;
  bool setMap(const std::vector<T> &map);
  void initMap() { map_.resize(totalSize()); }
  void initMapValue(const T &val) { map_.resize(totalSize(), val); }
  void initZeroMap() { map_.resize(totalSize(), 0); }
  bool setMapValue(const std::vector<double> &datam, const T &val) { return setMapValueCell(meters2cells(datam), val); }
  bool setMapValueCell(const std::vector<int> &datac, const T &val) {
    return inMapCell(datac) && (map_[subv2ind(datac)] = val);
  }
  bool setMapValueInd(size_t ind, const T &val) { return (ind < map_.size()) && (map_[ind] = val); }

  size_t subv2ind(const std::vector<int> &cell) const {
    return row_major_ ? subv2ind_rowmajor(cell) : subv2ind_colmajor(cell);
  }

  std::vector<int> ind2subv(size_t lindix) const {
    return row_major_ ? ind2subv_rowmajor(lindix) : ind2subv_colmajor(lindix);
  }

  template <typename Vec>
  bool inMap(const Vec &datam) const {
    for (size_t k = 0; k < datam.size(); ++k)
      if (datam[k] < min_[k] || datam[k] > max_[k]) return false;
    return true;
  }
  bool inMapCell(const std::vector<int> &datac) const {
    for (size_t k = 0; k < datac.size(); ++k)
      if (datac[k] < 0 || datac[k] >= size_[k]) return false;
    return true;
  }

  bool isFree(const std::vector<double> &datam) const { return isFreeCell(meters2cells(datam)); }
  bool isFreeCell(const std::vector<int> &datac) const { return inMapCell(datac) && (map_[subv2ind(datac)] <= T(0)); }

  std::vector<int> meters2cells(const std::vector<double> &datam) const { return erl::meters2cells(datam, min_, res_); }
  std::vector<double> cells2meters(const std::vector<int> &datac) const { return erl::cells2meters(datac, min_, res_); }
  std::vector<double> meters2cells_cont(const std::vector<double> &datam) const {
    return erl::meters2cells_cont(datam, min_, res_);
  }

  std::vector<int> meters2cells(const std::vector<double>::const_iterator &datam_begin,
                                const std::vector<double>::const_iterator &datam_end) const {
    return erl::meters2cells(datam_begin, datam_end, min_.begin(), min_.end(), res_.begin(), res_.end());
  }

  std::vector<double> cells2meters(const std::vector<int>::const_iterator &datac_begin,
                                   const std::vector<int>::const_iterator &datac_end) const {
    return erl::cells2meters(datac_begin, datac_end, min_.begin(), min_.end(), res_.begin(), res_.end());
  }

  std::vector<double> meters2cells_cont(const std::vector<double>::const_iterator &datam_begin,
                                        const std::vector<double>::const_iterator &datam_end) const {
    return erl::meters2cells_cont(datam_begin, datam_end, min_.begin(), min_.end(), res_.begin(), res_.end());
  }

  // Row major order as in C++
  size_t subv2ind_rowmajor(const std::vector<int> &datac) const;
  std::vector<int> ind2subv_rowmajor(size_t ind) const;

  // Column major order as in MATLAB
  std::size_t subv2ind_colmajor(std::vector<int> const &datac) const;
  std::vector<int> ind2subv_colmajor(std::size_t ind) const;

  GridMap<T> get2DSlice(size_t z_dim) const;
  GridMap<T> getSlice(const std::vector<unsigned> &dims_to_remove, const std::vector<unsigned> &vals_at_removed) const;

  void toColmajor();
  void toRowmajor();

  /**
   * Check if line segment [start, end] intersects with the grid map
   *
   * @param start line segment start point (meters)
   * @param end line segment end point (meters)
   * @param intersection_point point where the first intersection from start occurs
   */
  bool intersectSegment(const std::vector<double> &start, const std::vector<double> &end,
                        std::vector<double> &intersection_point) const;

  /**
   * Check if point y is visible from sensor pose (p,R) and is within the field of view
   * specified by range_bracket, azimuth_bracket, and elevation_bracket
   *
   * @param y query point in 2D
   * @param R sensor orientation (Forward-Left-UP frame; not optical frame)
   * @param p sensor position in 2D
   * @param range_bracket sensor [min_range, max_range]
   * @param azimuth_bracket sensor [min_azimuth, max_azimuth]
   */
  bool isPointVisibleFromPose(const Eigen::Vector2d &y, const Eigen::Matrix2d &R, const Eigen::Vector2d &p,
                              const Eigen::Vector2d &range_bracket, const Eigen::Vector2d &azimuth_bracket);

  /**
   * Check if point y is visible from sensor pose (p,R) and is within the field of view
   * specified by range_bracket, azimuth_bracket, and elevation_bracket
   *
   * @param y query point
   * @param R sensor orientation (Forward-Left-UP frame; not optical frame)
   * @param p sensor position
   * @param range_bracket sensor [min_range, max_range]
   * @param azimuth_bracket sensor [min_azimuth, max_azimuth]
   * @param elevation_bracket sensor [min_elevation, max_elevation]
   */
  bool isPointVisibleFromPose(const Eigen::Vector3d &y, const Eigen::Matrix3d &R, const Eigen::Vector3d &p,
                              const Eigen::Vector2d &range_bracket, const Eigen::Vector2d &azimuth_bracket,
                              const Eigen::Vector2d &elevation_bracket);

 private:
  void initMapData1(unsigned numdim, const double *mapmin, const double *mapmax, const double *mapres, bool rowmajor);
  void initMapData2(unsigned numdim, const int *mapdim, const double *origin, const double *mapres, bool rowmajor);
};

template <class T>
std::vector<T> inflateMap2D(const GridMap<T> &map, double rad) {
  std::vector<T> omap(map.map());
  if (rad < 0.00001) return omap;  // nothing to do here

  int bbx = static_cast<int>(std::ceil(rad / map.res()[0]));
  int bby = static_cast<int>(std::ceil(rad / map.res()[1]));

  std::vector<std::array<int, 2>> ns;
  ns.reserve((2 * bbx + 1) * (2 * bby + 1));
  for (int nx = -bbx; nx <= bbx; ++nx)
    for (int ny = -bby; ny <= bby; ++ny) {
      double d = std::hypot(nx * map.res()[0], ny * map.res()[1]);
      if (0 < d && d <= rad) ns.push_back({nx, ny});
    }

  for (auto x = 0; x < map.size()[0]; ++x)
    for (auto y = 0; y < map.size()[1]; ++y) {
      if (map.map()[map.subv2ind({(int)x, (int)y})] > 0) {  // If the cell is occupied,
        // Set its neighbors to 1.
        for (const auto &it : ns) {
          int nx = x + it[0], ny = y + it[1];
          if (0 <= nx && nx < map.size()[0] && 0 <= ny && ny < map.size()[1]) omap[map.subv2ind({nx, ny})] = 1;
        }
      }
    }
  return omap;
}

}  // namespace erl

//////////////////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////////////////

template <typename T>
inline std::size_t erl::GridMap<T>::totalSize() const {
  std::size_t sz = 1;
  for (unsigned k = 0; k < size_.size(); ++k) sz *= size_[k];
  return sz;
}

template <typename T>
inline bool erl::GridMap<T>::setMap(const std::vector<T> &map) {
  if (map.size() != totalSize()) return false;
  map_ = map;
  return true;
}

template <typename T>
inline size_t erl::GridMap<T>::subv2ind_rowmajor(const std::vector<int> &datac) const {
  const size_t ndims = datac.size();
  if (ndims == 0) return -1;

  size_t idx = datac.back();
  size_t prod = 1;
  for (int k = ndims - 1; k > 0; --k) {
    prod = prod * size_[k];
    idx += datac[k - 1] * prod;
  }
  return idx;
}

template <typename T>
inline std::vector<int> erl::GridMap<T>::ind2subv_rowmajor(size_t ind) const {
  const size_t ndims = size_.size();
  std::vector<int> subv(ndims);
  for (int k = ndims - 1; k >= 0; --k) {
    subv[k] = ind % size_[k];
    ind -= subv[k];
    ind /= size_[k];
  }
  return subv;
}

template <typename T>
inline std::size_t erl::GridMap<T>::subv2ind_colmajor(std::vector<int> const &datac) const {
  const size_t ndims = datac.size();
  if (ndims == 0) return -1;

  size_t idx = datac[0];
  size_t prod = 1;
  for (size_t k = 1; k < ndims; ++k) {
    prod *= size_[k - 1];
    idx += datac[k] * prod;
  }
  return idx;
}

template <typename T>
inline std::vector<int> erl::GridMap<T>::ind2subv_colmajor(std::size_t ind) const {
  const size_t ndims = size_.size();
  std::vector<int> subv(ndims);
  for (size_t k = 0; k < ndims; ++k) {
    subv[k] = ind % size_[k];
    ind -= subv[k];
    ind /= size_[k];
  }
  return subv;
}


template <typename T>
inline void erl::GridMap<T>::toColmajor() {
  if (!row_major_) return;
  const size_t mapsz = map_.size();
  std::vector<T> newmap(mapsz);
  for (size_t k = 0; k < mapsz; ++k) newmap[subv2ind_colmajor(ind2subv_rowmajor(k))] = map_[k];
  map_ = newmap;
}

template <typename T>
inline void erl::GridMap<T>::toRowmajor() {
  if (row_major_) return;
  const size_t mapsz = map_.size();
  std::vector<T> newmap(mapsz);
  for (size_t k = 0; k < mapsz; ++k) newmap[subv2ind_rowmajor(ind2subv_colmajor(k))] = map_[k];
  map_ = newmap;
}

template <typename T>
inline void erl::GridMap<T>::initMapData1(unsigned numdim, const double *mapmin, const double *mapmax,
                                          const double *mapres, bool rowmajor) {
  res_.assign(mapres, mapres + numdim);
  double dev;
  for (unsigned k = 0; k < numdim; ++k) {
    origin_.push_back((mapmin[k] + mapmax[k]) / 2);
    size_.push_back(erl::odd_ceil((mapmax[k] - mapmin[k]) / mapres[k]));
    dev = size_[k] * mapres[k] / 2;
    min_.push_back(origin_[k] - dev);
    max_.push_back(origin_[k] + dev);
    origincells_.push_back((size_[k] - 1) / 2);
  }
  row_major_ = rowmajor;
}

template <typename T>
inline void erl::GridMap<T>::initMapData2(unsigned numdim, const int *mapdim, const double *origin,
                                          const double *mapres, bool rowmajor) {
  size_.assign(mapdim, mapdim + numdim);
  origin_.assign(origin, origin + numdim);
  res_.assign(mapres, mapres + numdim);
  double dev;
  for (unsigned k = 0; k < numdim; ++k) {
    if (size_[k] % 2 == 0) size_[k] += 1;  // ensure the map size is odd!
    dev = size_[k] * res_[k] / 2;
    min_.push_back(origin_[k] - dev);
    max_.push_back(origin_[k] + dev);
    origincells_.push_back((size_[k] - 1) / 2);
  }
  row_major_ = rowmajor;
}

template <typename T>
inline erl::GridMap<T> erl::GridMap<T>::get2DSlice(size_t z_dim) const {
  // If the Map is already 2D, we don't need to do anything.
  if (size().size() == 2) {
    return *this;
  } else {
    erl::GridMap<T> slice(std::vector<int>({size_[0], size_[1]}), std::vector<double>({origin_[0], origin_[1]}),
                          std::vector<double>({res_[0], res_[1]}), row_major_);
    std::vector<T> slice_map(size_[0] * size_[1]);

    if (!row_major_) {  // For Column-major order
      std::copy(map_.begin() + z_dim * slice_map.size(), map_.begin() + (z_dim + 1) * slice_map.size(),
                slice_map.begin());
    } else {  // For Row-major order
      for (size_t k = 0; k < slice_map.size(); k++) {
        auto cell = slice.ind2subv(k);                           // Get Cell coordinates of the 2D Point
        auto result = subv2ind({cell[0], cell[1], (int)z_dim});  // Lookup in 3D cost-map
        slice_map[k] = map_[result];
      }
    }
    slice.setMap(slice_map);
    return slice;
  }
}


#endif
