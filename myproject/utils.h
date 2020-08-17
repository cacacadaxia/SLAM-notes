//
// Created by cacacadaxia on 2020/8/13.
//

#ifndef VO1_UTILS_H
#define VO1_UTILS_H

#include "Common.h"


namespace Utils{
    void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps) {
        ifstream fAssociation;
        fAssociation.open(strAssociationFilename.c_str());
        while (!fAssociation.eof()) {
            string s;
            getline(fAssociation, s);
            if (!s.empty()) {
                stringstream ss;
                ss << s;
                double t;
                string sRGB, sD;
                ss >> t;
                vTimestamps.push_back(t);
                ss >> sRGB;
                vstrImageFilenamesRGB.push_back(sRGB);
                ss >> t;
                ss >> sD;
                vstrImageFilenamesD.push_back(sD);
            }
        }
    }
}


#endif //VO1_UTILS_H
