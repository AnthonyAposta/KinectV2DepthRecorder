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

    void keyPressed(int key) override;

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
    ofMesh pointCloud2;


    void convert32BitTo3Channel8bit(unsigned long int* in, int in_size, unsigned char* out);
    void convert3Channel8bitTo32bit(unsigned char* in, int in_size, unsigned long int* out);
    void testConversion32bitTo3C8bit();
    void getPointXYZ(unsigned long int* frame, int r, int c, float& x, float& y, float& z);
    void fillVboMesh(unsigned long int* depthFrame, ofPixels rgbRegiteredPixels, int h, int w);
    void cropDepthData(unsigned long int* depthData);
    void updateFromKinect();
    void updateFromVideoFile();
    void drawDebugScren();
    void drawPointCloud();
    void updateFromVideoFileOnSaving();

    ofImage depthOFImage;
    ofImage RGBOFImage;
    ofTexture RGBImage;

    ofFloatPixels RawPixelsFloat;
    ofPixels pixelsFromDepthOFImage;

    ofxSpout::Sender spoutSenderDepth;
    ofxSpout::Sender spoutSenderRGB;

    ofVideoPlayer depthVidPlayer;
    ofBoxPrimitive box;

    // define guis
    ofxPanel recordingGui;
    ofxPanel playbackGui;

    ofParameter<unsigned short> minDistanceBits;
    ofParameter<unsigned short> maxDistanceBits;
    ofParameter<float> NearMeters;
    ofParameter<float> FarMeters;
    
    ofParameter<float> HighMeters;
    ofParameter<float> LowMeters;
    
    ofParameter<float> LeftMeters;
    ofParameter<float> RightMeters;


    // Screen controls variables
    bool showDebugScreen = false;
    bool onRecording = false;
    bool showPointCloud = true;
    bool showPlaybackScreen = false;


    std::vector<glm::vec3> points;
    unsigned char* depthFrameRGB;
    unsigned long int* RawPixelsInt;
    unsigned char* RawPixelsChar;
    unsigned long int* revertedRawPixelsInt;
    int h;
    int w;
    bool onSavingObj;
    ofFileDialogResult dialogResult;
    ofFileDialogResult SaveObjDialog;
    string obj_folder_path;
    string obj_filename;


};