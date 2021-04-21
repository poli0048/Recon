//
//  Image.m
//  DroneClient
//
//  Created by Ben Choi on 4/11/21.
//

#include "Image.hpp"


Image::Image(unsigned char * bitmap, int rows, int cols, int size_pixel) {
    this->bitmap = bitmap;
    this->rows = rows;
    this->cols = cols;
    this->size_pixel = size_pixel;
}
