#include "createFile.h"

void createFile(std::vector<std::vector<double> >& container, bool type)
{
    std::string fileName = "ballData.CSV";
    if(type)
        fileName = "cupData.CSV";

    std::ofstream file(fileName);
    for(const std::vector<double>& innerVector : container)
    {
        for(const double value : innerVector)
        {
            file << value << " ";
        }
        file << std::endl;
    }
    file.close();
    std::cout << "File: " << fileName << " was succesfully written" << std::endl;
}
