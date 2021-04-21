//
//  Image.h
//  DroneClient
//
//  Created by Ben Choi on 4/11/21.
//


class Image {
    public:
        Image() = default;
        ~Image() = default;
        Image(unsigned char * bitmap, int rows, int cols, int size_pixel);
    
        unsigned char* bitmap;
        int rows;
        int cols;
        int size_pixel;
};
