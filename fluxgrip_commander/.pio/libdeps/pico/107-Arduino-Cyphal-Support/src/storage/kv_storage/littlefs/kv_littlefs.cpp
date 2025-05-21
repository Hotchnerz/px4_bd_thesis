/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/107-Arduino-Cyphal-Support/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "kv_littlefs.h"

#if !defined(__GNUC__) || (__GNUC__ >= 11)

#include <map>
#include <string>
#include <variant>
#include <sstream>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace cyphal::support::platform::storage::littlefs
{

/**************************************************************************************
 * PRIVATE FREE FUNCTIONS
 **************************************************************************************/

[[nodiscard]] static inline std::string toFilename(std::string_view const key)
{
  auto const key_hash = std::hash<std::string_view>{}(key);
  std::stringstream key_filename;
  key_filename << key_hash;
  return key_filename.str();
}

[[nodiscard]] static inline Error toError(::littlefs::Error const err)
{
  static std::map<::littlefs::Error, Error> const LITTLEFS_TO_STORAGE_ERROR_MAP =
  {
    {::littlefs::Error::IO          , Error::IO},
    {::littlefs::Error::CORRUPT     , Error::Internal},
    {::littlefs::Error::NOENT       , Error::Existence},
    {::littlefs::Error::EXIST       , Error::Existence},
    {::littlefs::Error::NOTDIR      , Error::Existence},
    {::littlefs::Error::ISDIR       , Error::Existence},
    {::littlefs::Error::NOTEMPTY    , Error::Existence},
    {::littlefs::Error::BADF        , Error::Internal},
    {::littlefs::Error::FBIG        , Error::Capacity},
    {::littlefs::Error::INVAL       , Error::API},
    {::littlefs::Error::NOSPC       , Error::Capacity},
    {::littlefs::Error::NOMEM       , Error::Internal},
    {::littlefs::Error::NOATTR      , Error::API},
    {::littlefs::Error::NAMETOOLONG , Error::API},
    {::littlefs::Error::NO_FD_ENTRY , Error::API},
  };

  return LITTLEFS_TO_STORAGE_ERROR_MAP.at(err);
}

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

KeyValueStorage::KeyValueStorage(::littlefs::Filesystem & filesystem)
: _filesystem{filesystem}
{ }

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

auto KeyValueStorage::get(const std::string_view key, const std::size_t size, void* const data) const
  -> std::variant<Error, std::size_t>
{
  auto const rc_open = _filesystem.open(toFilename(key), ::littlefs::OpenFlag::RDONLY);
  if (const auto * const err = std::get_if<::littlefs::Error>(&rc_open))
    return toError(*err);

  auto const file_hdl = std::get<::littlefs::FileHandle>(rc_open);

  auto const rc_read = _filesystem.read(file_hdl, data, size);
  if (const auto * const err = std::get_if<::littlefs::Error>(&rc_read))
  {
    (void)_filesystem.close(file_hdl);
    return toError(*err);
  }

  (void)_filesystem.close(file_hdl);

  return std::get<size_t>(rc_read);
}

auto KeyValueStorage::put(const std::string_view key, const std::size_t size, const void* const data)
  -> std::optional<Error>
{
  auto const rc_open = _filesystem.open(toFilename(key), ::littlefs::OpenFlag::WRONLY | ::littlefs::OpenFlag::CREAT | ::littlefs::OpenFlag::TRUNC);
  if (const auto * const err = std::get_if<::littlefs::Error>(&rc_open))
    return toError(*err);

  auto const file_hdl = std::get<::littlefs::FileHandle>(rc_open);

  auto const rc_write = _filesystem.write(file_hdl, data, size);
  if (const auto * const err = std::get_if<::littlefs::Error>(&rc_write))
  {
    (void)_filesystem.close(file_hdl);
    return toError(*err);
  }

  (void)_filesystem.close(file_hdl);

  size_t const bytes_written = std::get<size_t>(rc_write);
  if (bytes_written != size)
    return Error::Capacity;

  return std::nullopt;
}

auto KeyValueStorage::drop(const std::string_view key) -> std::optional<Error>
{
  if (auto const opt_err = _filesystem.remove(toFilename(key)); opt_err.has_value())
    return toError(opt_err.value());

  return std::nullopt;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* cyphal::support::platform::storage::littlefs */

#endif /* !defined(__GNUC__) || (__GNUC__ >= 11) */
