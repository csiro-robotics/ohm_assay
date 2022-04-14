// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "PointCloud2Helper.h"

#include <string>
#include <unordered_map>

namespace ohmdataros
{
using PointAttributeMap = std::unordered_map<std::string, sensor_msgs::PointField>;

struct PointCloud2HelperDetail
{
  sensor_msgs::PointField time;
  sensor_msgs::PointField x;
  sensor_msgs::PointField y;
  sensor_msgs::PointField z;
  sensor_msgs::PointField intensity;
  sensor_msgs::PointField azimuth;
  sensor_msgs::PointField elevation;
  sensor_msgs::PointField range;
  sensor_msgs::PointField encoder;
  sensor_msgs::PointField returnNumber;
  sensor_msgs::PointField ring;
  sensor_msgs::PointField red;
  sensor_msgs::PointField green;
  sensor_msgs::PointField blue;

  PointAttributeMap attributes;

  bool fromBigEndian = false;

  void clear()
  {
    time = x = y = z = intensity = azimuth = elevation = range = sensor_msgs::PointField();
    encoder = returnNumber = ring = sensor_msgs::PointField();
    red = green = blue = sensor_msgs::PointField();
    attributes.clear();
  }
};
}  // namespace ohmdataros

namespace
{
bool mapField(sensor_msgs::PointField &field, const ohmdataros::PointAttributeMap &attributes, const std::string &name)
{
  const auto search = attributes.find(name);
  if (search == attributes.end())
  {
    // Clear field.
    field.name.clear();
    field.offset = 0;
    field.datatype = 0;
    field.count = 0;
    return false;
  }

  field = search->second;
  return true;
}

bool mapField(sensor_msgs::PointField &field, const ohmdataros::PointAttributeMap &attributes,
              const std::vector<std::string> names)
{
  for (auto &&name : names)
  {
    if (mapField(field, attributes, name))
    {
      return true;
    }
  }

  return false;
}

template <typename T>
T value(const uint8_t *pointStruct, const sensor_msgs::PointField &field, bool fromBigEndian, unsigned index)
{
  // Data field sizes corresponding to sensor_msgs::PointField types.
  static const unsigned FieldWidth[9] = { 0, 1, 1, 2, 2, 4, 4, 4, 8 };
  const uint8_t *valuePtr = pointStruct + field.offset;

  // Buffer for dealing with endian swap.
  uint8_t swapBuffer[8];
  const unsigned dataWidth = FieldWidth[field.datatype];

  // Clamp the index.
  index = (index < field.count) ? index : field.count - 1;

  // Marshal value into a swap buffer for endian changes.
  for (unsigned i = 0; i < dataWidth; ++i)
  {
    swapBuffer[i] = valuePtr[dataWidth * index + i];
  }

  if (ohmdataros::PointCloud2Helper::isBigEndianHost() != fromBigEndian)
  {
    for (unsigned i = 0; i < dataWidth / 2; ++i)
    {
      std::swap(swapBuffer[i], swapBuffer[dataWidth - i - 1]);
    }
  }

  const uint8_t *ptr = swapBuffer;
  switch (field.datatype)
  {
  case 0:
    return T(0);
  case sensor_msgs::PointField::INT8:
    return T(*(const int8_t *)ptr);
  case sensor_msgs::PointField::UINT8:
    return T(*(const uint8_t *)ptr);
  case sensor_msgs::PointField::INT16:
    return T(*(const int16_t *)ptr);
  case sensor_msgs::PointField::UINT16:
    return T(*(const uint16_t *)ptr);
  case sensor_msgs::PointField::INT32:
    return T(*(const int32_t *)ptr);
  case sensor_msgs::PointField::UINT32:
    return T(*(const uint32_t *)ptr);
  case sensor_msgs::PointField::FLOAT32:
    return T(*(const float *)ptr);
  case sensor_msgs::PointField::FLOAT64:
    return T(*(const double *)ptr);

  default:
    break;
  }

  return T(0);
}


float colourChannel(float float_value, const sensor_msgs::PointField &field)
{
  switch (field.datatype)
  {
  case sensor_msgs::PointField::INT8:
    return float_value / float(std::numeric_limits<int8_t>::max());
  case sensor_msgs::PointField::UINT8:
    return float_value / float(std::numeric_limits<uint8_t>::max());
  case sensor_msgs::PointField::INT16:
    return float_value / float(std::numeric_limits<int16_t>::max());
  case sensor_msgs::PointField::UINT16:
    return float_value / float(std::numeric_limits<uint16_t>::max());
  case sensor_msgs::PointField::INT32:
    return float_value / float(std::numeric_limits<int32_t>::max());
  case sensor_msgs::PointField::UINT32:
    return float_value / float(std::numeric_limits<uint32_t>::max());
  default:
    break;
  }

  return float_value;
}
}  // namespace

namespace ohmdataros
{
PointCloud2Helper::PointCloud2Helper()
  : imp_(std::make_unique<PointCloud2HelperDetail>())
{
  imp_->fromBigEndian = isBigEndianHost();
}


PointCloud2Helper::PointCloud2Helper(const sensor_msgs::PointField *fields, unsigned fieldCount, bool fromBigEndian,
                                     const FieldNames *names)
  : PointCloud2Helper()
{
  init(fields, fieldCount, fromBigEndian, names);
}


PointCloud2Helper::~PointCloud2Helper() = default;


bool PointCloud2Helper::initialised() const
{
  return !imp_->attributes.empty();
}


bool PointCloud2Helper::init(const sensor_msgs::PointField *fields, unsigned fieldCount, bool fromBigEndian,
                             const FieldNames *names)
{
  static const FieldNames defaultNames;
  if (!names)
  {
    names = &defaultNames;
  }

  imp_->fromBigEndian = fromBigEndian;

  // Build the overall field mapping.
  imp_->attributes.clear();

  for (unsigned i = 0; i < fieldCount; ++i)
  {
    imp_->attributes[fields[i].name] = fields[i];
  }

  // Resolve mappings.
  mapField(imp_->time, imp_->attributes, names->time);

  mapField(imp_->x, imp_->attributes, names->x);
  mapField(imp_->y, imp_->attributes, names->y);
  mapField(imp_->z, imp_->attributes, names->z);

  mapField(imp_->azimuth, imp_->attributes, names->azimuth);
  mapField(imp_->elevation, imp_->attributes, names->elevation);
  mapField(imp_->range, imp_->attributes, names->range);

  mapField(imp_->intensity, imp_->attributes, names->intensity);

  mapField(imp_->encoder, imp_->attributes, names->encoder);

  mapField(imp_->returnNumber, imp_->attributes, names->returnNumber);
  mapField(imp_->ring, imp_->attributes, names->ring);

  mapField(imp_->red, imp_->attributes, names->red);
  mapField(imp_->green, imp_->attributes, names->green);
  mapField(imp_->blue, imp_->attributes, names->blue);

  if (!cartesian() && !polar())
  {
    clear();
    return false;
  }

  return true;
}


bool PointCloud2Helper::cartesian() const
{
  return xCount() && yCount() && zCount();
}


bool PointCloud2Helper::polar() const
{
  return azimuthCount() && rangeCount();
}


bool PointCloud2Helper::colour() const
{
  return redCount() && greenCount() && blueCount();
}


void PointCloud2Helper::clear()
{
  imp_->clear();
}


bool PointCloud2Helper::validate(const sensor_msgs::PointField *fields, unsigned fieldCount, bool fromBigEndian) const
{
  if (fieldCount != imp_->attributes.size())
  {
    return false;
  }

  if (fromBigEndian != imp_->fromBigEndian)
  {
    return false;
  }

  for (unsigned i = 0; i < fieldCount; ++i)
  {
    if (imp_->attributes.find(fields[i].name) == imp_->attributes.end())
    {
      return false;
    }
  }

  return true;
}

bool PointCloud2Helper::nextPoint(const uint8_t **currentPoint, const sensor_msgs::PointCloud2 *scan)
{
  if (!*currentPoint)
  {
    if (!scan->data.empty() && scan->height * scan->width != 0)
    {
      *currentPoint = scan->data.data();
      return true;
    }
    return false;
  }

  // Resolve the current column index.
  const uint8_t *scan_data = scan->data.data();
  const size_t row_index = (*currentPoint - scan_data) / scan->row_step;
  const uint8_t *row_ptr = scan_data + row_index * scan->row_step;
  const size_t col_index = (*currentPoint - row_ptr) / scan->point_step;

  // Try increment to next column.
  if (col_index + 1 < scan->width)
  {
    // Increment by a point.
    *currentPoint += scan->point_step;
  }
  // Try increment to next row.
  else if (row_index + 1 < scan->height)
  {
    *currentPoint = row_ptr + scan->row_step;
  }
  // All done.
  else
  {
    *currentPoint = nullptr;
  }

  return *currentPoint;
}


bool PointCloud2Helper::isBigEndianHost()
{
  union EndianTest
  {
    uint32_t i;
    char c[4];
  };

  const static EndianTest testVal = { 0x01020304 };
  return testVal.c[0] == 1;
}


unsigned PointCloud2Helper::timeCount() const
{
  return imp_->time.count;
}


double PointCloud2Helper::time(const uint8_t *pointStruct, unsigned index) const
{
  return value<double>(pointStruct, imp_->time, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::xCount() const
{
  return imp_->x.count;
}


double PointCloud2Helper::x(const uint8_t *pointStruct, unsigned index) const
{
  return value<double>(pointStruct, imp_->x, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::yCount() const
{
  return imp_->y.count;
}


double PointCloud2Helper::y(const uint8_t *pointStruct, unsigned index) const
{
  return value<double>(pointStruct, imp_->y, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::zCount() const
{
  return imp_->z.count;
}


double PointCloud2Helper::z(const uint8_t *pointStruct, unsigned index) const
{
  return value<double>(pointStruct, imp_->z, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::azimuthCount() const
{
  return imp_->azimuth.count;
}


double PointCloud2Helper::azimuth(const uint8_t *pointStruct, unsigned index) const
{
  return value<double>(pointStruct, imp_->azimuth, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::elevationCount() const
{
  return imp_->elevation.count;
}


double PointCloud2Helper::elevation(const uint8_t *pointStruct, unsigned index) const
{
  return value<double>(pointStruct, imp_->elevation, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::rangeCount() const
{
  return imp_->range.count;
}


double PointCloud2Helper::range(const uint8_t *pointStruct, unsigned index) const
{
  return value<double>(pointStruct, imp_->range, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::intensityCount() const
{
  return imp_->intensity.count;
}


double PointCloud2Helper::intensity(const uint8_t *pointStruct, unsigned index) const
{
  return value<double>(pointStruct, imp_->intensity, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::encoderCount() const
{
  return imp_->encoder.count;
}


double PointCloud2Helper::encoder(const uint8_t *pointStruct, unsigned index) const
{
  return value<double>(pointStruct, imp_->encoder, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::returnNumberCount() const
{
  return imp_->returnNumber.count;
}


int PointCloud2Helper::returnNumber(const uint8_t *pointStruct, unsigned index) const
{
  return value<int>(pointStruct, imp_->returnNumber, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::ringCount() const
{
  return imp_->ring.count;
}


int PointCloud2Helper::ring(const uint8_t *pointStruct, unsigned index) const
{
  return value<int>(pointStruct, imp_->ring, imp_->fromBigEndian, index);
}


unsigned PointCloud2Helper::hasAttribute(const char *name) const
{
  sensor_msgs::PointField attr;
  mapField(attr, imp_->attributes, name);
  return (attr.datatype) ? std::min<uint32_t>(1, attr.count) : 0u;
}


const sensor_msgs::PointField &PointCloud2Helper::attribute(const char *name) const
{
  const auto search = imp_->attributes.find(name);
  if (search == imp_->attributes.end())
  {
    static const sensor_msgs::PointField nullField;
    return nullField;
  }

  return search->second;
}


unsigned PointCloud2Helper::redCount() const
{
  return imp_->red.count;
}


float PointCloud2Helper::red(const uint8_t *pointStruct, unsigned index) const
{
  return colourChannel(value<float>(pointStruct, imp_->red, imp_->fromBigEndian, index), imp_->red);
}

unsigned PointCloud2Helper::greenCount() const
{
  return imp_->green.count;
}


float PointCloud2Helper::green(const uint8_t *pointStruct, unsigned index) const
{
  return colourChannel(value<float>(pointStruct, imp_->green, imp_->fromBigEndian, index), imp_->green);
}


unsigned PointCloud2Helper::blueCount() const
{
  return imp_->blue.count;
}


float PointCloud2Helper::blue(const uint8_t *pointStruct, unsigned index) const
{
  return colourChannel(value<float>(pointStruct, imp_->blue, imp_->fromBigEndian, index), imp_->blue);
}


double PointCloud2Helper::attributeDouble(const char *name, const uint8_t *pointStruct, unsigned index) const
{
  sensor_msgs::PointField attr;
  mapField(attr, imp_->attributes, name);
  return value<double>(pointStruct, attr, imp_->fromBigEndian, index);
}


int32_t PointCloud2Helper::attributeInt32(const char *name, const uint8_t *pointStruct, unsigned index) const
{
  sensor_msgs::PointField attr;
  mapField(attr, imp_->attributes, name);
  return value<int32_t>(pointStruct, attr, imp_->fromBigEndian, index);
}


int64_t PointCloud2Helper::attributeInt64(const char *name, const uint8_t *pointStruct, unsigned index) const
{
  sensor_msgs::PointField attr;
  mapField(attr, imp_->attributes, name);
  return value<int64_t>(pointStruct, attr, imp_->fromBigEndian, index);
}
}  // namespace ohmdataros
