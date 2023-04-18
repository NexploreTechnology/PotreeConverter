#ifndef NGP_LAS_POINT_WRITER_H
#define NGP_LAS_POINT_WRITER_H

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

// const double kColorEigenToLas = 65535.0;

class NGP_LASPointWriter {

public:
    NGP_LASPointWriter() {}

    ~NGP_LASPointWriter() {}

    void writePoints(std::filesystem::path path, AABB aabb, double scale, std::vector<Point> points)
    {
        // create writer
        laszip_POINTER laszip_writer;
        if (laszip_create(&laszip_writer)) {
            return;
        }

        // set header
        laszip_header* header;
        if (laszip_get_header_pointer(laszip_writer, &header)) {
            laszip_close_writer(laszip_writer);
            laszip_destroy(laszip_writer);
            return;
        }

        // populate header
        // rely on range in cloud
        {
            memset(header, 0, sizeof(laszip_header));
            strcpy(header->generating_software, "NexGeometryProcessing");
            header->version_major = 1;
            header->version_minor = 2;
            header->header_size = 227;

            header->min_x = aabb.min.x;
            header->min_y = aabb.min.y;
            header->min_z = aabb.min.z;
            header->max_x = aabb.max.x;
            header->max_y = aabb.max.y;
            header->max_z = aabb.max.z;

            header->x_offset = aabb.min.x;
            header->y_offset = aabb.min.y;
            header->z_offset = aabb.min.z;

            header->x_scale_factor = scale;
            header->y_scale_factor = scale;
            header->z_scale_factor = scale;

            header->number_of_point_records = points.size();

            header->offset_to_point_data = header->header_size;
            header->number_of_variable_length_records = 0;
            header->vlrs = NULL;

            header->point_data_format = 2;
            header->point_data_record_length = 26;
        }

        // open writer
        laszip_BOOL is_compressed = false;
        if (path.extension().u8string() == ".laz") {
            is_compressed = true;
        }

        if (laszip_open_writer(laszip_writer, path.u8string().c_str(), is_compressed)) {
            char* tmp;
            laszip_get_error(laszip_writer, &tmp);
            cout << "Laszip error: " << tmp << endl;
            laszip_destroy(laszip_writer);
            return;
        }

        // position and color
        for (auto p : points)
        {
            laszip_point* point_write;

            if (laszip_get_point_pointer(laszip_writer, &point_write)) {
                continue;
            }

            // copy point structure data
            (*point_write).X = std::round((p.position.x - header->x_offset) / header->x_scale_factor);
            (*point_write).Y = std::round((p.position.y - header->y_offset) / header->y_scale_factor);
            (*point_write).Z = std::round((p.position.z - header->z_offset) / header->z_scale_factor);

            // get color
            (*point_write).rgb[0] = p.color.x; // * kColorEigenToLas;
            (*point_write).rgb[1] = p.color.y; // * kColorEigenToLas;
            (*point_write).rgb[2] = p.color.z; // * kColorEigenToLas;

            // update classification
            (*point_write).classification = p.classification;

            // update intensity
            (*point_write).intensity = p.intensity;

            if (laszip_set_point(laszip_writer, point_write)) {
            }

            if (laszip_write_point(laszip_writer)) {
            }
        }

        laszip_close_writer(laszip_writer);
        laszip_destroy(laszip_writer);
    }

};

}

#endif
