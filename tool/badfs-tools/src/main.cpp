#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <print>
#include <string>
#include <vector>

struct file_info {
  std::string absolute_path;
  std::string bfs_path;
  uint32_t size, size_aligned;
};

struct badfs_file {
  uint32_t offset;
  uint32_t size;
  char path[64];
};

struct badfs_header {
  uint32_t magic;
  uint32_t total_size;
  uint32_t num_files;
};

static_assert(sizeof(badfs_file) == 72, "ding dong your compiler is wrong");
static_assert(sizeof(badfs_header) == 12, "ding dong your compiler is wrong");

int main(int argc, char *argv[]) {

  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <input_directory> <output_file>"
              << std::endl;
    return 1;
  }

  std::string input_directory = argv[1];
  std::string output_file = argv[2];
  input_directory = std::filesystem::absolute(input_directory).string();

  // 0. Open output file
  std::ofstream ofs(output_file, std::ios::binary | std::ios::trunc);

  if (!ofs) {
    std::println(std::cerr, "Error: Could not open output file for writing.");
    return 1;
  }

  // 1. Find all files
  uint32_t blob_size = 0;
  std::vector<file_info> files;

  for (const auto &entry :
       std::filesystem::recursive_directory_iterator(input_directory)) {
    if (entry.is_regular_file()) {
      file_info info;
      info.absolute_path = entry.path().string();
      info.bfs_path =
          "/" +
          std::filesystem::relative(entry.path(), input_directory).string();
      info.size = static_cast<uint32_t>(entry.file_size());
      info.size_aligned = (info.size + 3) & ~3; // Align to 4 bytes
      blob_size += info.size_aligned;
      files.push_back(info);
    }
  }

  // 2. Write header + metadata
  uint32_t metadata_size = sizeof(badfs_file) * files.size();
  uint32_t total_size = sizeof(badfs_header) + metadata_size + blob_size;
  badfs_header header;
  header.magic = 0xBADF5;
  header.total_size = total_size;
  header.num_files = static_cast<uint32_t>(files.size());

  ofs.write(reinterpret_cast<const char *>(&header), sizeof(header));

  // 3. Compute offsets & write file entries
  uint32_t cur_offset = sizeof(badfs_header) + metadata_size;

  for (const auto &file : files) {
    badfs_file metadata;
    metadata.offset = cur_offset;
    metadata.size = file.size;

    if (file.bfs_path.size() >= sizeof(metadata.path)) {
      std::println(std::cerr,
                 "Error: File path '{}' is too long for the badfs format (max "
                 "{} characters).",
                 file.bfs_path, sizeof(metadata.path) - 1);
      return 1;
    }

    std::strncpy(metadata.path, file.bfs_path.c_str(),
                 sizeof(metadata.path) - 1);
    metadata.path[file.bfs_path.size()] = '\0'; // Ensure null-termination

    ofs.write(reinterpret_cast<const char *>(&metadata), sizeof(metadata));
    cur_offset += file.size_aligned;
  }

  // 4. Copy over file data

  for (const auto &file : files) {
    std::ifstream ifs(file.absolute_path, std::ios::binary);
    if (!ifs) {
      std::println(std::cerr, "Error: Could not open file '{}' for reading.",
                 file.absolute_path);
      return 1;
    }

    std::vector<char> buffer(file.size);
    ifs.read(buffer.data(), file.size);
    ofs.write(buffer.data(), file.size);

    // Write padding if necessary
    uint32_t padding_size = file.size_aligned - file.size;
    for (uint32_t i = 0; i < padding_size; ++i) {
      ofs.put(0);
    }
  }

  ofs.close();

  std::println("Created badfs image '{}' with {} files ({} bytes).", output_file,
             files.size(), total_size);
}