#include "../src/logger.hpp"

class Two {

    public: 
        Two() {

        }

        void WritetoLog() {
            Logger::Get().log("THIS IS A NEEEEEWWWW TEST!!!!!");
            Logger::Get().logErr("ERRORRRRRRRR");
            Logger::Get().printLog();
        }

        void TryPrint() {
            Logger::Get().printLog();
        }
};

class One {

    public: 
        Two two;
        One() {
            two = Two();
        }

        void OpenLog() {
            Logger::Get().openLog("output.txt");
            two.WritetoLog();
            Logger::Get().closeLog();
            two.TryPrint();
        }
};




int main(int argc, char* argv[]) {

    // Logger::Get().openLog("output.txt");
    // Logger::Get().log("Testing Info aksdjflkas lksajfdlasjfaowe aslkdjf alskjf asodfj03945430 0349534 5039485 345");
    // Logger::Get().log("Testing Info Just some stuff");
    // Logger::Get().log("Testing something words words words");
    // Logger::Get().logErr("Testing Error lksdj fa 3459374");
    // Logger::Get().log("Testing blah blah blah blah blah lbal lab lbas labal abala");
    // Logger::Get().log("Testing Info aksdjflkas l11111111111111111111111111111111111111111485 345");
    // Logger::Get().logErr("Testing Error Some other type of error");
    // Logger::Get().logErr("Testing Error Another one");
    // Logger::Get().closeLog();
    // //log.printLog();

    One one = One();
    one.OpenLog();

}