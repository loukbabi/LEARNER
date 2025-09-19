#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "Usage: ./convert_vocab input.yml output.xml.gz" << std::endl;
        return 1;
    }

    std::string input_path = argv[1];
    std::string output_path = argv[2];

    cv::FileStorage fs_in(input_path, cv::FileStorage::READ);
    if (!fs_in.isOpened()) {
        std::cerr << "Failed to open input vocabulary file: " << input_path << std::endl;
        return 1;
    }

    cv::Mat vocab;
    fs_in["vocabulary"] >> vocab;
    fs_in.release();

    cv::FileStorage fs_out(output_path, cv::FileStorage::WRITE);
    fs_out << "vocabulary" << vocab;
    fs_out.release();

    std::cout << "Vocabulary successfully converted to: " << output_path << std::endl;
    return 0;
}

