#include "src/DBoW3.h"
#include <iostream>


int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: ./voc_to_bin input.yaml output.dbow3\n";
        return 1;
    }

    std::string input_yaml = argv[1];
    std::string output_bin = argv[2];

    std::cout << "Loading vocabulary using OpenCV YAML parser...\n";

    DBoW3::Vocabulary voc;
    voc.load(input_yaml);   // <- this works for modern nested YAML

    std::cout << "Saving as binary .dbow3...\n";
    voc.save(output_bin);

    std::cout << "Done: " << output_bin << "\n";
    return 0;
}

