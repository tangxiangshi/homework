#include <iostream>
#include <fstream>

int main( int argc, char* argv[])
{
      std::ofstream myfile;
      myfile.open ("/home/ee405423/Desktop/test.csv");
      for(int i =0 ;i<5;i++)
      {
        myfile << i <<"," <<i+10 << "\n";
        myfile.close();
      }
      //myfile.close();
      return 0;
}
