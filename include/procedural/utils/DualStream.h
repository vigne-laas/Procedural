#ifndef PROCEDURAL_DUALSTREAM_H
#define PROCEDURAL_DUALSTREAM_H

#include <cstdio>
#include <iostream>
#include <fstream>
#include <streambuf>
#include <string>

// #ifndef EOF
//  #define EOF (-1) // EOF is not defined maybe due to antlr4
// #endif


namespace procedural {
class DualStream : public std::ostream {
public:
    DualStream(std::ostream& stream1, std::ostream& stream2) : std::ostream(&buffer), buffer(stream1, stream2) {}

private:
    class Buffer : public std::streambuf {
    public:
        Buffer(std::ostream& stream1, std::ostream& stream2) : stream1_(stream1.rdbuf()), stream2_(stream2.rdbuf()) {}

        virtual int overflow(int c)
        {
            // if (c != EOF) {
            //     if (stream1_->sputc(c) == EOF)
            //         return EOF;
            //     if (stream2_->sputc(c) == EOF)
            //         return EOF;
            // }
            return c;
        }

        virtual int sync()
        {
            if (stream1_->pubsync() == -1)
                return -1;
            if (stream2_->pubsync() == -1)
                return -1;
            return 0;
        }

    private:
        std::streambuf* stream1_;
        std::streambuf* stream2_;
    };

    Buffer buffer;
};
}; // procedural
#endif //PROCEDURAL_DUALSTREAM_H
