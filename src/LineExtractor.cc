#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "LineExtractor.h"

using namespace cv;
using namespace line_descriptor;
using namespace std;

namespace ORB_SLAM2
{

Lineextractor::Lineextractor(int _lsd_nfeatures, double _llength_th, bool _bFLD) 
    :lsd_nfeatures(_lsd_nfeatures), min_line_length(_llength_th), bFLD(_bFLD)
{

}

Lineextractor::Lineextractor(int _lsd_nfeatures, double _llength_th, int _lsd_refine, double _lsd_scale, double _lsd_sigma_scale, 
    double _lsd_quant, double _lsd_ang_th, double _lsd_log_eps, double _lsd_density_th, int _lsd_n_bins, bool _bFLD)
    :lsd_nfeatures(_lsd_nfeatures), min_line_length(_llength_th), lsd_refine(_lsd_refine), lsd_scale(_lsd_scale),
    lsd_sigma_scale(_lsd_sigma_scale), lsd_quant(_lsd_quant), lsd_ang_th(_lsd_ang_th), lsd_log_eps(_lsd_log_eps), 
    lsd_density_th(_lsd_density_th), lsd_n_bins(_lsd_n_bins), bFLD(_bFLD)
{

}

void Lineextractor::operator()( const cv::Mat& img, const cv::Mat& mask, 
            std::vector<cv::line_descriptor::KeyLine>& keylines, cv::Mat& descriptors_line)
{
    // Detect line features
    keylines.clear();
    Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
    if( Config::hasLines() )
    {

        if( !bFLD )
        {
            Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();
            // lsd parameters
            line_descriptor::LSDDetectorC::LSDOptions opts;
            opts.refine       = lsd_refine;
            opts.scale        = lsd_scale;
            opts.sigma_scale  = lsd_sigma_scale;
            opts.quant        = lsd_quant;
            opts.ang_th       = lsd_ang_th;
            opts.log_eps      = lsd_log_eps;
            opts.density_th   = lsd_density_th;
            opts.n_bins       = lsd_n_bins;
            opts.min_length   = min_line_length*(std::min(img.cols,img.rows));
            lsd->detect( img, keylines, lsd_scale, 1, opts);
            // filter keylines
            if( int(keylines.size())>lsd_nfeatures && lsd_nfeatures!=0  )
            {
                // sort keylines by their response
                sort( keylines.begin(), keylines.end(), sort_lines_by_response() );
                //sort( keylines.begin(), keylines.end(), sort_lines_by_length() );
                keylines.resize(lsd_nfeatures);
                // reassign index
                for( int i = 0; i < lsd_nfeatures; i++  )
                    keylines[i].class_id = i;
            }
            lbd->compute( img, keylines, descriptors_line);
        }
        else return;
        /*
        else
        {
            Mat fld_img, img_gray;
            vector<Vec4f> fld_lines;

            if( img.channels() != 1 )
            {
                cv::cvtColor( img, img_gray, CV_RGB2GRAY );
                img_gray.convertTo( fld_img, CV_8UC1 );
            }
            else
                img.convertTo( fld_img, CV_8UC1 );

            Ptr<cv::ximgproc::FastLineDetector> fld = cv::ximgproc::createFastLineDetector(min_line_length);
            fld->detect( fld_img, fld_lines );

            // filter keylines
            if( int(fld_lines.size())>lsd_nfeatures && lsd_nfeatures!=0  )
            {
                // sort keylines by their response
                sort( fld_lines.begin(), fld_lines.end(), sort_flines_by_length() );
                fld_lines.resize(lsd_nfeatures);
            }

            // loop over keylines object transforming into a vector<KeyLine>
            keylines.reserve(fld_lines.size());
            for( unsigned int i = 0; i < fld_lines.size(); i++ )
            {
                KeyLine kl;
                double octaveScale = 1.f;
                int    octaveIdx   = 0;

                kl.startPointX     = fld_lines[i][0] * octaveScale;
                kl.startPointY     = fld_lines[i][1] * octaveScale;
                kl.endPointX       = fld_lines[i][2] * octaveScale;
                kl.endPointY       = fld_lines[i][3] * octaveScale;

                kl.sPointInOctaveX = fld_lines[i][0];
                kl.sPointInOctaveY = fld_lines[i][1];
                kl.ePointInOctaveX = fld_lines[i][2];
                kl.ePointInOctaveY = fld_lines[i][3];

                kl.lineLength = (float) sqrt( pow( fld_lines[i][0] - fld_lines[i][2], 2 ) + pow( fld_lines[i][1] - fld_lines[i][3], 2 ) );

                kl.angle    = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
                kl.class_id = i;
                kl.octave   = octaveIdx;
                kl.size     = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
                kl.pt       = Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

                kl.response = kl.lineLength / max( fld_img.cols, fld_img.rows );
                cv::LineIterator li( fld_img, Point2f( fld_lines[i][0], fld_lines[i][1] ), Point2f( fld_lines[i][2], fld_lines[i][3] ) );
                kl.numOfPixels = li.count;

                keylines.push_back( kl );

            }

            // compute lbd descriptor
            lbd->compute( fld_img, keylines, descriptors_line);
        }
        */
    }
}

} //namespace ORB_SLAM
