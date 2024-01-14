#pragma once


#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGui.h"


class ofApp : public ofBaseApp
{
public:
    void setup() override;
    void update() override;
    void draw() override;

    //void keyPressed(int key) override;

    void drawTextureAtRowAndColumn(const std::string& title,
        const ofTexture& tex,
        int row,
        int column);

    ofxPanel panel;

    std::vector<std::shared_ptr<ofxKinectV2>> kinects;

    std::vector<ofTexture> texRGB;
    std::vector<ofTexture> texRGBRegistered;
    std::vector<ofTexture> texIR;
    std::vector<ofTexture> texDepth;

    std::size_t currentKinect = 0;

    int numRows = 2;
    int numColumns = 2;

    ofEasyCam cam;
    ofMesh pointCloud;
    bool showPointCloud = false;

    void convert16BitTo2Channel8bit(unsigned short* in, int in_size, unsigned char* out); // not used
    void convert16BitTo3Channel8bit(unsigned short* in, int in_size, unsigned char* out);
    void convert32BitTo3Channel8bit(unsigned long int* in, int in_size, unsigned char* out);
    void convert3Channel8bitTo32bit(unsigned char* in, int in_size, unsigned long int* out);
    void testConversion32bitTo3C8bit();

    unsigned char* depthFrameRGB;
    ofImage depthOFImage;
    unsigned long int* RawPixelsInt;
    ofFloatPixels RawPixelsFloat;
    unsigned char* RawPixelsChar;
    unsigned long int* revertedRawPixelsInt;
    ofPixels pixelsFromDepthOFImage;

    void keyReleased(int key);
    int GMult;
    int GMult2;
    int GMult3;

};