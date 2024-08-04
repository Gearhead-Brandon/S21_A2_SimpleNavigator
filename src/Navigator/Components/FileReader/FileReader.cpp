/**
 * @file FileReader.cpp
 * @brief Implementation of the class FileReader
 */

#include "FileReader.h"

#include <iostream>

namespace s21 {

/**
 * @brief Parametrized constructor
 * @param path - path to the file
 */
FileReader::FileReader(const std::string &path) : file(path) {}

/**
 * @brief Destructor
 */
FileReader::~FileReader() {
  if (file.is_open()) file.close();
}

/**
   * @brief Check if the file is open
   * @return true if the file is open, false otherwise
  */
bool FileReader::IsFileOpen() { return file.is_open(); }
}  // namespace s21