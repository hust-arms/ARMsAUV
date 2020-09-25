#ifndef FILE_WRITER_H_
#define FILE_WRITER_H_

#include <fstream>
#include <exception>
#include <string>

/**
 * @brief Writing data in target file
 */ 
class FileWriter{
public:
    /**
     * @brief Constructor
     * @param filename Name of file
     * @param path Path of file
     */ 
    FileWriter(const std::string& filename, const std::string& path);

    /**
     * @brief Constructor
     * @param filename Name of file
     */ 
    FileWriter(const std::string& filename);

    /**
     * @brief Deconstructor
     */ 
    ~FileWriter(){file_s_.clear(); file_s_.close();};

    /**
     * @brief Wrtie data into file
     * @param flag Mode of writing
     */ 
    bool writeData(const std::string& data);

private:
    std::fstream file_s_; // file pointer
    std::string filename_;
    std::string file_;
    std::string path_; 

}; // for FileWriter

#endif
