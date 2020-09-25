#include <armsauv_docking/file_writer.h>
#include <ios>
#include <armsauv_docking/common.h>

FileWriter::FileWriter(const std::string& filename, const std::string& path) : filename_(filename), path_(path)
{
    file_ = path_ + filename_;
}

FileWriter::FileWriter(const std::string& filename) : filename_(filename){
    file_ = filename_;
}

bool FileWriter::writeData(const std::string& data){
    /* Open file */
    // contentPrint<std::string>("file_writer", "Try open file");
    file_s_.open(file_, std::ios::app);
    file_s_.precision(4);
    file_s_.setf(std::ios_base::showpoint);

    std::string data_copy = data;

    /* If file is not opened, create file */
    if(!file_s_.is_open()){
        // contentPrint<std::string>("file_writer", "Create file");
        file_s_.clear();
        file_s_.open(file_, std::ios::out); // create
	file_s_.close();
	file_s_.open(file_, std::ios::app);
    }
    else{
        data_copy = '\n' + data;
    }

    /* Add data */
    try{
        file_s_ << data_copy;
	file_s_.close();
	return true;
    }
    catch(std::exception& ex){
	file_s_.close();
        return false;
    }
}

