#ifndef NGP_LAS_POINT_READER_H
#define NGP_LAS_POINT_READER_H

#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <stdio.h>
#include <string.h>

#include "laszip_api.h"

#include "AABB.h"
#include "PointWriter.hpp"
#include "Point.h"
#include "stuff.h"

using std::string;
using std::fstream;
using std::ios;


namespace Potree{

// const double kColorLasToEigen = 1.0 / 65535.0;

class NGP_LASPointReader {

public:
    NGP_LASPointReader() {}

    ~NGP_LASPointReader() {}

    std::vector<Point> readPoints(std::filesystem::path path)
    {
        std::vector<Point> points;

        // check file exists
        if (!std::filesystem::exists(path)) {
            std::cout << "ERROR file not found " << path << std::endl;
            return points;
        }

        // create reader
        laszip_POINTER laszip_reader;
        if (laszip_create(&laszip_reader)) {
            std::cout << "ERROR laszip_create " << path << std::endl;
            return points;
        }

        // open reader
        laszip_BOOL is_compressed = 0;
        if (laszip_open_reader(laszip_reader, path.u8string().c_str(), &is_compressed)) {
            laszip_destroy(laszip_reader);
            std::cout << "ERROR laszip_open_reader " << path << std::endl;
            return points;
        }

        // get header
        laszip_header* header;
        if (laszip_get_header_pointer(laszip_reader, &header)) {
            laszip_close_reader(laszip_reader);
            laszip_destroy(laszip_reader);
            std::cout << "ERROR laszip_get_header_pointer " << path << std::endl;
            return points;
        }

        // get num points
        laszip_I64 npoints = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

        // get points
        laszip_point* point_read;
        if (laszip_get_point_pointer(laszip_reader, &point_read)) {
            laszip_close_reader(laszip_reader);
            laszip_destroy(laszip_reader);
            std::cout << "ERROR laszip_get_point_pointer " << path << std::endl;
            return points;
        }

        // calc centroid, bounding box as we go
        // Eigen::Vector3d centroid(0.0, 0.0, 0.0);

        for (size_t i = 0; i < static_cast<size_t>(npoints); i++)
        {
            Point p;

            // read a point
            if (laszip_read_point(laszip_reader)) {
                continue;
            }

            p.position.x = header->x_scale_factor * point_read->X + header->x_offset;
            p.position.y = header->y_scale_factor * point_read->Y + header->y_offset;
            p.position.z = header->z_scale_factor * point_read->Z + header->z_offset;

            p.color.x = point_read->rgb[0]; // *= kColorLasToEigen;
            p.color.y = point_read->rgb[1]; // *= kColorLasToEigen;
            p.color.z = point_read->rgb[2]; // *= kColorLasToEigen;

            p.classification = point_read->classification;

            p.intensity = point_read->intensity;

            points.push_back(p);
        }

        // close the reader
        if (laszip_close_reader(laszip_reader))
        {
        }

        // destroy the reader
        if (laszip_destroy(laszip_reader))
        {
        }

        return points;
    }

};

}

#endif
