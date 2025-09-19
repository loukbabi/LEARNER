#include "DBoW3.h"
#include <iostream>

int main(int argc, char** argv) {
    if(argc < 2) {
        std::cerr << "Usage: " << argv[0] << " path_to_vocab\n";
        return -1;
    }

    DBoW3::Vocabulary voc;
    bool loaded = voc.load(argv[1]);
    if(!loaded) {
        std::cerr << "Failed to load vocabulary file: " << argv[1] << "\n";
        return -1;
    }

    std::cout << "Vocabulary loaded successfully!\n";
    std::cout << "Vocabulary size: " << voc.size() << "\n";
    return 0;
}

