/*!
 * \file dsst_tracker.h
 *
 * \author Pei
 * \date ���� 2017
 *
 * 
 */

#ifndef DLIB_SSDT_TRACKER_H_
#define DLIB_SSDT_TRACKER_H_

#include "../correlationtracker_export.h"
#include "dlib/geometry.h"
#include "dlib/matrix.h"
#include "dlib/array2d.h"
#include "dlib/image_transforms/assign_image.h"
#include <dlib/gui_widgets.h>
#include <dlib/opencv.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <omp.h>

using namespace std;

typedef dlib::cv_image<unsigned char> image_type;

namespace dlib
{

// ----------------------------------------------------------------------------------------

class dsst_tracker
{
  public:
    dsst_tracker();

    void init();

    void start_track(
        const image_type &img,
        const dlib::drectangle &p);

    unsigned long get_filter_size() const; // must be power of 2

    unsigned long get_num_scale_levels() const; // must be power of 2

    unsigned long get_scale_window_size() const;

    double get_regularizer_space() const;
    double get_nu_space() const;

    double get_regularizer_scale() const;
    double get_nu_scale() const;

    dlib::drectangle get_position() const;

    std::vector<dlib::drectangle> get_position_part() const;

    double get_scale_pyramid_alpha() const;

    double update(
        const image_type &img,
        const dlib::drectangle &guess);
    double update(
        const image_type &img);

  private:
    void make_scale_space(
        const image_type &img,
        std::vector<dlib::matrix<std::complex<double>, 0, 1>> &Fs) const;

    dlib::point_transform_affine make_chip(
        const image_type &img,
        drectangle p,
        std::vector<dlib::matrix<std::complex<double>>> &chip) const;

    void make_target_location_image(
        const dlib::vector<double, 2> &p,
        dlib::matrix<std::complex<double>> &g) const;

    void make_scale_target_location_image(
        const double scale,
        dlib::matrix<std::complex<double>, 0, 1> &g) const;

    dlib::matrix<double> make_cosine_mask() const;

    std::vector<dlib::matrix<std::complex<double>>> A, F;
    dlib::matrix<double> B;

    std::vector<dlib::matrix<std::complex<double>, 0, 1>> As, Fs;
    dlib::matrix<double, 0, 1> Bs;

    dlib::drectangle position;

    dlib::matrix<double> mask;
    std::vector<double> scale_cos_mask;

    // G and Gs do not logically contribute to the state of this object.  They are
    // here just so we can void reallocating them over and over.
    dlib::matrix<std::complex<double>> G;
    dlib::matrix<std::complex<double>, 0, 1> Gs;
};
}

#endif // DLIB_CORRELATION_TrACKER_H_