// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMDATAROS_POINTCLOUD2HELPER_H
#define OHMDATAROS_POINTCLOUD2HELPER_H

#include "ohmdataros/DataRosConfig.h"

#include <sensor_msgs/PointCloud2.h>

#include <memory>

namespace ohmdataros
{
/// List of property/field names to try for various point cloud properties.
///
/// This structure defines what the expected field name(s) for known PLY properties are in a @c sensor_msgs::PointCloud2
/// message. For example, the @p time member defaults to looking for "time", "times" or "timestamps" in that order. The
/// @c PointCloud2Helper will attempt to resolve the actual time field as the first field in the
/// @c sensor_msgs::PointCloud2 message which matches a @p time value candidate.
struct FieldNames
{
  /// Time field names.
  std::vector<std::string> time = { "time", "times", "timestamp" };

  /// Cartesian X coordinate name.
  std::string x = "x";
  /// Cartesian Y coordinate name.
  std::string y = "y";
  /// Cartesian Z coordinate name.
  std::string z = "z";

  /// Polar azimuth name.
  std::vector<std::string> azimuth = { "azimuth", "angles", "angle" };
  /// Polar elevation name.
  std::vector<std::string> elevation = { "elevation", "inclination" };
  /// Range field name.
  std::vector<std::string> range = { "distance", "ranges", "range" };

  /// Encoder signal
  std::vector<std::string> encoder = { "encoder" };

  /// Point intensity name.
  std::vector<std::string> intensity = { "intensity", "intensities" };

  /// Return number of multi-return lidar.
  std::vector<std::string> returnNumber = { "returnNum", "returnNumber", "return" };
  /// Ring number.
  std::vector<std::string> ring = { "ring" };

  /// Red colour channel name.
  std::vector<std::string> red = { "red", "r" };
  /// Green colour channel name.
  std::vector<std::string> green = { "green", "g" };
  /// Blue colour channel name.
  std::vector<std::string> blue = { "b", "b" };
};

struct PointCloud2HelperDetail;

/// Type traints for resolving data type for writing point cloud fields.
template <typename T>
struct FieldInfo
{
  /// Return the @p sensor_msgs::PointField::datatype value for type @c T .
  /// @return The data type of @c T .
  constexpr static uint8_t datatype() = delete;
  /// Return the byte size type @c T .
  /// @return The byte size of @c T .
  constexpr static size_t size() = delete;
};

/// Specialisation for @c int8_t .
template <>
struct FieldInfo<int8_t>
{
  constexpr static uint8_t datatype() { return sensor_msgs::PointField::INT8; }
  constexpr static size_t size() { return 1; }
};

/// Specialisation for @c uint8_t .
template <>
struct FieldInfo<uint8_t>
{
  constexpr static uint8_t datatype() { return sensor_msgs::PointField::UINT8; }
  constexpr static size_t size() { return 1; }
};

/// Specialisation for @c int16_t .
template <>
struct FieldInfo<int16_t>
{
  constexpr static uint8_t datatype() { return sensor_msgs::PointField::INT16; }
  constexpr static size_t size() { return 2; }
};

/// Specialisation for @c uint16_t .
template <>
struct FieldInfo<uint16_t>
{
  constexpr static uint8_t datatype() { return sensor_msgs::PointField::UINT16; }
  constexpr static size_t size() { return 2; }
};

/// Specialisation for @c int32_t .
template <>
struct FieldInfo<int32_t>
{
  constexpr static uint8_t datatype() { return sensor_msgs::PointField::INT32; }
  constexpr static size_t size() { return 4; }
};

/// Specialisation for @c uint32_t .
template <>
struct FieldInfo<uint32_t>
{
  constexpr static uint8_t datatype() { return sensor_msgs::PointField::UINT32; }
  constexpr static size_t size() { return 4; }
};

/// Specialisation for @c float (float 32).
template <>
struct FieldInfo<float>
{
  constexpr static uint8_t datatype() { return sensor_msgs::PointField::FLOAT32; }
  constexpr static size_t size() { return 4; }
};

/// Specialisation for @c double (float 64).
template <>
struct FieldInfo<double>
{
  constexpr static uint8_t datatype() { return sensor_msgs::PointField::FLOAT64; }
  constexpr static size_t size() { return 8; }
};

/// Get the data size of @p field . Considers data type size and field count.
/// @param field The field data.
inline uint32_t fieldSize(const sensor_msgs::PointField &field)
{
  uint32_t field_size = 0;
  switch (field.datatype)
  {
  case sensor_msgs::PointField::INT8:
  case sensor_msgs::PointField::UINT8:
    field_size = 1;
    break;
  case sensor_msgs::PointField::INT16:
  case sensor_msgs::PointField::UINT16:
    field_size = 2;
    break;
  case sensor_msgs::PointField::INT32:
  case sensor_msgs::PointField::UINT32:
  case sensor_msgs::PointField::FLOAT32:
    field_size = 4;
    break;
  case sensor_msgs::PointField::FLOAT64:
    field_size = 8;
    break;
  default:
    break;
  }

  return field_size * field.count;
}

/// Get the offset starting after the given @p field .
inline uint32_t nextOffset(const sensor_msgs::PointField &field)
{
  return field.offset + fieldSize(field);
}

/// Add a point cloud message field.
/// @param fields Field to add to
/// @param name Name of the field.
/// @param count Number of elements of type @c T in the field.
/// @tparam T The data type for the field.
template <typename T>
size_t addField(std::vector<sensor_msgs::PointField> &fields, const std::string &name, uint32_t count = 1)
{
  sensor_msgs::PointField field;
  field.name = name;
  field.offset = (!fields.empty()) ? ohmdataros::nextOffset(fields.back()) : 0u;
  field.count = count;
  field.datatype = ohmdataros::FieldInfo<T>::datatype();
  fields.emplace_back(field);
  return fields.size() - 1u;
}

/// Write data to a @c PointCloud2 payload for the field at @p field_index .
///
/// This utility function a value for writes the @p value for a @c sensor_msgs::PointCloud2 field to a section of the
/// @c sensor_msgs::PointCloud2::data .
///
/// See @c appendPoint() for usage.
///
/// @param addr The address for the point to write data for. Generally the return value from @c appendPoint()
/// @param fields The data fields for the @c sensor_msgs::PointCloud2 message assumed in
/// @c sensor_msgs::PointField::offset order . The overall size must match the size available at @p addr .
/// @param field_index Index of the field for which we are to write @p value . Must be in range `[0, fields.size())`
/// @param value The Value for the @p field_index . May be cast to the true data type of @c fields[field_index]
/// using a simple @c static_cast operation.
/// @return True if the field is for a known type.
template <typename T>
bool writeField(uint8_t *addr, const std::vector<sensor_msgs::PointField> &fields, size_t field_index, const T &value)
{
  // Resolve the local offset to write to.
  const uint32_t offset = (field_index > 0) ? nextOffset(fields[field_index - 1]) : 0;
  switch (fields[field_index].datatype)
  {
  case sensor_msgs::PointField::INT8:
    (*reinterpret_cast<int8_t *>(addr + offset)) = static_cast<int8_t>(value);
    break;
  case sensor_msgs::PointField::UINT8:
    (*reinterpret_cast<uint8_t *>(addr + offset)) = static_cast<uint8_t>(value);
    break;
  case sensor_msgs::PointField::INT16:
    (*reinterpret_cast<int16_t *>(addr + offset)) = static_cast<int16_t>(value);
    break;
  case sensor_msgs::PointField::UINT16:
    (*reinterpret_cast<uint16_t *>(addr + offset)) = static_cast<uint16_t>(value);
    break;
  case sensor_msgs::PointField::INT32:
    (*reinterpret_cast<int32_t *>(addr + offset)) = static_cast<int32_t>(value);
    break;
  case sensor_msgs::PointField::UINT32:
    (*reinterpret_cast<uint32_t *>(addr + offset)) = static_cast<uint32_t>(value);
    break;
  case sensor_msgs::PointField::FLOAT32:
    (*reinterpret_cast<float *>(addr + offset)) = static_cast<float>(value);
    break;
  case sensor_msgs::PointField::FLOAT64:
    (*reinterpret_cast<double *>(addr + offset)) = static_cast<double>(value);
    break;
  default:
    return false;
  }

  return true;
}

/// Append a point to a @c sensor_msgs::PointCloud2 payload.
///
/// This increments the @c data array size adding sufficient space for an additional point as determined by the
/// @p fields array and returns an address to the new point memory. Note the @p fields array is assumed to be ordered by
/// @p sensor_msgs::PointField::offset .
///
/// Typical usage:
/// @code
/// sensor_msgs::PointCloud2 cloud;
/// cloud.header = /* Initialise header */;
///
/// cloud.height = 1;
/// cloud.width = 100;
///
/// // Add cartesian coordinates.
/// size_t x_idx = addField<float>(cloud.fields, "x");
/// size_t y_idx = addField<float>(cloud.fields, "y");
/// size_t z_idx = addField<float>(cloud.fields, "z");
///
/// for (unsigned i = 0; i < cloud.height * cloud.width; ++i)
/// {
///   // Add a point to the cloud data and cache the address.
///   uint8_t *point_address = appendPoint(cloud.data(), cloud.fields);
///   // below, getPoint[XYZ]f() are user defined functions which get the
///   // respective coordinate value for a point in single precision
///   writeField(point_address, cloud.fields, x_idx, getPointXf(i));
///   writeField(point_address, cloud.fields, y_idx, getPointYf(i));
///   writeField(point_address, cloud.fields, z_idx, getPointZf(i));
/// }
///
/// @endcode
/// @pa
inline uint8_t *appendPoint(std::vector<uint8_t> &data, const std::vector<sensor_msgs::PointField> &fields)
{
  // Resolve the data size of the fields.
  const size_t data_size = nextOffset(fields.back());
  const size_t initial_data_size = data.size();
  for (size_t i = 0; i < data_size; ++i)
  {
    data.emplace_back(0u);
  }
  return data.data() + initial_data_size;
}

/// A helper structure for interfacing with from a @c sensor_msgs::PointCloud2.
///
/// Typical read usage is to is as follows:
/// - Declare a @c PointCloud2Helper
/// - Receive the first @c sensor_msgs::PointCloud2 or otherwise identify the @c sensor_msgs::PointField layout.
/// - (Optional) Setup the expected field mappings with a custom @c FieldNames structure.
/// - Call @c init() with the @c sensor_msgs::PointField array.
/// - Initialise the @c current_point pointer to null, of type <tt>const uint8_t *</tt>.
/// - While @c nextPoint(current_point, scan) returns true:
///   - Call various field interface methods (@c x(), @c time(), @c intensity() passing @c current_point.
///
/// @code
/// void decodeScan(const sensor_msgs::PointCloud2::Ptr &scan)
/// {
///   PointCloud2Helper reader;
///
///   if (!reader.init(scan.fields.data, unsigned(scan.fields.size()), scan.isBigEndian))
///   {
///     std::cerr << "field resolution failed" << std::endl;
///     return;
///   }
///
///   const uint8_t *point = nullptr;
///   while (reader.nextPoint(&point, scan))
///   {
///     std::cout << "point: ";
///     std::cout << reader.x(point) << ',';
///     std::cout << reader.y(point) << ',';
///     std::cout << reader.z(point);
///     std::cout << std::endl;
///   }
/// }
/// @endcode
class PointCloud2Helper
{
public:
  /// Default constructor
  PointCloud2Helper();
  /// Construct with the given @c sensor_msgs::PointField data.
  ///
  /// Calls @c init()
  ///
  /// @param fields Array of fields to expect when reading.
  /// @param fieldCount Number of elements in @p fields
  /// @param fromBigEndian Are we reading big endian data?
  /// @param names Identifies which input field names to attempt to resolve. Uses the default @c FieldNames when
  /// omitted.
  PointCloud2Helper(const sensor_msgs::PointField *fields, unsigned fieldCount, bool fromBigEndian,
                    const FieldNames *names = nullptr);
  /// Destruct.r
  ~PointCloud2Helper();

  /// True if the reader has been initialised (and currently cleared).
  /// @return True if initialised.
  bool initialised() const;

  /// Initialise the helper to read the expected @p fields data.
  ///
  /// It's safe to reinitialise, though this may change the field mappings. It's recommended that the helper only be
  /// initialised once.
  ///
  /// @param fields Array of fields to expect when reading.
  /// @param fieldCount Number of elements in @p fields
  /// @param fromBigEndian Are we reading big endian data?
  /// @param names Identifies which input field names to attempt to resolve. Uses the default @c FieldNames when
  /// omitted.
  /// @return True when either @c cartesian() or @c polar() is true after resolving field names.
  bool init(const sensor_msgs::PointField *fields, unsigned fieldCount, bool fromBigEndian,
            const FieldNames *names = nullptr);

  /// Are cartesian coordinates (xyz) available.
  /// @return True when cartesian points are available.
  bool cartesian() const;

  /// Are polar coordinates (azimuth/range) available. Note elevation is optional.
  /// @return True when polar points are available.
  bool polar() const;

  /// Check if red/green/blue colour channels are all present.
  bool colour() const;

  /// Clear the reader content, allowing another call to @c init().
  void clear();

  /// Validate that the provided @c fields exactly match those stored in this reader.
  /// @param fields The field array to validate.
  /// @param fieldCount Number of elements in @p fields.
  /// @param fromBigEndian Source data is in big Endian format?
  /// @return True if all @p fields are present in the reader and there are not additional fields.
  bool validate(const sensor_msgs::PointField *fields, unsigned fieldCount, bool fromBigEndian) const;

  /// Adjust @p currentPoint to reference the next point in @p scan accounting for point and row strides.
  ///
  /// Sets @p *currentPoint to the first point from @p scan->data when @p *currentPoint is passed as null.
  ///
  /// @param currentPoint A pointer to the pointer to adjust to the next point in @p scan->data.
  /// @param scan The scan data which @p currentPoint references.
  /// @return True if so long as a next point is available and @p currentPoint references a valid point.
  static bool nextPoint(const uint8_t **currentPoint, const sensor_msgs::PointCloud2 *scan);

  /// @overload
  static inline bool nextPoint(const uint8_t **currentPoint, const sensor_msgs::PointCloud2::Ptr &scan)
  {
    return nextPoint(currentPoint, scan.get());
  }

  /// @overload
  static inline bool nextPoint(const uint8_t **currentPoint, const sensor_msgs::PointCloud2::ConstPtr &scan)
  {
    return nextPoint(currentPoint, scan.get());
  }

  static bool isBigEndianHost();

  /// Number of time values available.
  unsigned timeCount() const;
  double time(const uint8_t *pointStruct, unsigned index = 0) const;

  unsigned xCount() const;
  double x(const uint8_t *pointStruct, unsigned index = 0) const;
  unsigned yCount() const;
  double y(const uint8_t *pointStruct, unsigned index = 0) const;
  unsigned zCount() const;
  double z(const uint8_t *pointStruct, unsigned index = 0) const;

  unsigned azimuthCount() const;
  double azimuth(const uint8_t *pointStruct, unsigned index = 0) const;
  unsigned elevationCount() const;
  double elevation(const uint8_t *pointStruct, unsigned index = 0) const;
  unsigned rangeCount() const;
  double range(const uint8_t *pointStruct, unsigned index = 0) const;

  unsigned intensityCount() const;
  double intensity(const uint8_t *pointStruct, unsigned index = 0) const;

  unsigned encoderCount() const;
  double encoder(const uint8_t *pointStruct, unsigned index = 0) const;

  unsigned returnNumberCount() const;
  int returnNumber(const uint8_t *pointStruct, unsigned index = 0) const;
  unsigned ringCount() const;
  int ring(const uint8_t *pointStruct, unsigned index = 0) const;

  unsigned hasAttribute(const char *name) const;
  const sensor_msgs::PointField &attribute(const char *name) const;

  unsigned redCount() const;
  float red(const uint8_t *pointStruct, unsigned index = 0) const;
  unsigned greenCount() const;
  float green(const uint8_t *pointStruct, unsigned index = 0) const;
  unsigned blueCount() const;
  float blue(const uint8_t *pointStruct, unsigned index = 0) const;

  double attributeDouble(const char *name, const uint8_t *pointStruct, unsigned index = 0) const;
  int32_t attributeInt32(const char *name, const uint8_t *pointStruct, unsigned index = 0) const;
  int64_t attributeInt64(const char *name, const uint8_t *pointStruct, unsigned index = 0) const;

private:
  std::unique_ptr<PointCloud2HelperDetail> imp_;
};
}  // namespace ohmdataros

#endif  // OHMDATAROS_POINTCLOUD2HELPER_
