#include "ofApp.h"

//NOTE: if you are unable to connect to your device on OS X, try unplugging and replugging in the power, while leaving the USB connected.
//ofxKinectV2 will only work if the NUI Sensor shows up in the Superspeed category of the System Profiler in the USB section.

//On OS X if you are not using the example project. Make sure to add OpenCL.framework to the Link Binary With Library Build Phase 
//and change the line in Project.xcconfig to OTHER_LDFLAGS = $(OF_CORE_LIBS) $(OF_CORE_FRAMEWORKS) -framework OpenCL


void ofApp::setup()
{
    // Uncomment for verbose info from libfreenect2
    ofSetLogLevel(OF_LOG_VERBOSE);

    ofBackground(0);

    //see how many devices we have.
    ofxKinectV2 tmp;
    std::vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();

    //allocate for this many devices
    kinects.resize(deviceList.size());
    texDepth.resize(kinects.size());
    texRGB.resize(kinects.size());
    texRGBRegistered.resize(kinects.size());
    texIR.resize(kinects.size());

    panel.setup("", "settings.xml", 10, 100);

    ofxKinectV2::Settings ksettings;
    ksettings.enableRGB = true;
    ksettings.enableIR = true;
    ksettings.enableDepth = true;
    ksettings.enableRGBRegistration = true;
    ksettings.config.MinDepth = 0.5;
    ksettings.config.MaxDepth = 8.0;

    int h = 424;
    int w = 512;
    depthFrameRGB = new unsigned char[w * h * 3];
    revertedRawPixelsInt = new unsigned long int[h * w];

    //depthOFImage.allocate(w, h, OF_IMAGE_COLOR);

    //****************************************//
    // example settings for just getting the depth stream //
//    ksettings.enableRGB = false;
//    ksettings.enableRGBRegistration = false;
//    ksettings.enableIR = false;
//    // use enableDepthRegistration if you would like to use getWorldCoordinateAt
//    ksettings.enableDepthRegistration = true;
    //****************************************//

    // Note you don't have to use ofxKinectV2 as a shared pointer, but if you
    // want to have it in a vector ( ie: for multuple ) it needs to be.
    for (int d = 0; d < kinects.size(); d++) {
        kinects[d] = std::make_shared<ofxKinectV2>();
        kinects[d]->open(deviceList[d].serial, ksettings);
        panel.add(kinects[d]->params);
    }

    panel.loadFromFile("settings.xml");
    GMult = 1;
    GMult2 = 1;
    GMult3 = 1;

}


void ofApp::update()
{

    for (int d = 0; d < kinects.size(); d++)
    {
        kinects[d]->update();

        if (kinects[d]->isFrameNew())
        {
            if (kinects[d]->isRGBEnabled()) texRGB[d].loadData(kinects[d]->getPixels());
            if (kinects[d]->getRegisteredPixels().getWidth() > 10) texRGBRegistered[d].loadData(kinects[d]->getRegisteredPixels());
            if (kinects[d]->isIREnabled()) texIR[d].loadData(kinects[d]->getIRPixels());
            //if (kinects[d]->isDepthEnabled()) texDepth[d].loadData(kinects[d]->getDepthPixels());

            if (kinects[d]->isDepthEnabled()) {

                
                RawPixelsFloat = kinects[d]->getRawDepthPixels();
                RawPixelsInt = reinterpret_cast<unsigned long int*>(RawPixelsFloat.getData());
          
                int h = RawPixelsFloat.getHeight();
                int w = RawPixelsFloat.getWidth();

                this->convert32BitTo3Channel8bit(RawPixelsInt, w * h, depthFrameRGB);
                depthOFImage.setFromPixels(depthFrameRGB, w, h, OF_IMAGE_COLOR);


                // Reversion
                //pixelsFromDepthOFImage = depthOFImage.getPixels();
                //this->convert3Channel8bitTo32bit(pixelsFromDepthOFImage.getData(), h * w, revertedRawPixelsInt);
                //cout << RawPixelsInt[1000] << "," << revertedRawPixelsInt[1000] << endl;

                texDepth[d].loadData(depthOFImage.getPixels());


            }


            if (showPointCloud)
            {
                pointCloud.clear();
                for (std::size_t x = 0; x < texRGBRegistered[d].getWidth(); x++)
                {
                    for (std::size_t y = 0; y < texRGBRegistered[d].getHeight(); y++)
                    {
                        pointCloud.addVertex(kinects[d]->getWorldCoordinateAt(x, y));
                        pointCloud.addColor(kinects[d]->getRegisteredPixels().getColor(x, y));
                    }
                }
            }
        }
    }
}


void ofApp::keyReleased(int key) {
    //
    //	Multiplier of the green channel for the depth video:

    if (key == 't') {
        GMult++;
        std::cout << "channel 1 multiplier: " << GMult << std::endl;
    }
    else if (key == 'y') {
        if (GMult > 1) {
            GMult--;
            std::cout << "channel 1 multiplier: " << GMult << std::endl;
        }
    }
    else if (key == 'b') {
        GMult2++;
        std::cout << "channel 2 multiplier: " << GMult2 << std::endl;
    }

    else if (key == 'n') {
        if (GMult2 > 1) {
            GMult2--;
            std::cout << "channel 2 multiplier: " << GMult2 << std::endl;
        }
    }
    else if (key == 'g') {
        GMult3++;
        std::cout << "channel 3 multiplier: " << GMult3 << std::endl;
    }
    else if (key == 'h') {
        if (GMult3 > 1) {
            GMult3--;
            std::cout << "channel 3 multiplier: " << GMult3 << std::endl;
        }
    }

}

void ofApp::convert16BitTo2Channel8bit(unsigned short* in, int in_size, unsigned char* out)
{
    for (int i = 0; i < in_size; i++) {
        out[i * 2] = static_cast<unsigned char>(in[i] & 0xFF); // lower byte
        out[i * 2 + 1] = static_cast<unsigned char>((in[i] >> 8) & 0xFF); // higher byte
    }
}
void ofApp::convert16BitTo3Channel8bit(unsigned short* in, int in_size, unsigned char* out)
{
    
    for (int i = 0; i < in_size; i++) {
        out[i * 3] = static_cast<unsigned char>((in[i]) & 0xFF); // lower byte
        out[i * 3 + 1] = GMult * static_cast<unsigned char>((in[i] >> 8) & 0xFF); // higher byte
        out[i * 3 + 2] = 0;
    }
}

void ofApp::convert32BitTo3Channel8bit(unsigned long int* in, int in_size, unsigned char* out)
{
    // Input has 8 bytes, only last two ar used.
    // The output order will be (2th byte, 3th byte, 0)
 
    for (int i = 0; i < in_size; i++) {
        
        out[i * 3 + 0] = GMult * static_cast<unsigned char>(((in[i]>>16) & 0xFF)); // lower byte
        // 0 to 69
        out[i * 3 + 1] = GMult2 * static_cast<unsigned char>((in[i] >> 24) & 0xFF); // higher byte
        out[i * 3 + 2] = 0;
    }

}

void ofApp::convert3Channel8bitTo32bit(unsigned char* in, int out_size, unsigned long int* out) {

    for (int i = 0; i < out_size; i++) {
        out[i] = static_cast<unsigned long int>(in[i * 3] << 16 | in[i * 3 + 1] << 24 | 0 | 0);
    }
    
}



void ofApp::testConversion32bitTo3C8bit() {

    // 32 bit from kinect
    unsigned long int inputKinect[2] = { (unsigned long int)4294901760, (unsigned long int)(4294901760) };
    cout << "kinect 32bits input " << inputKinect[0] << "," << inputKinect[1] << endl;


    // convert into 3 channel 8 bit
    unsigned char* outkinect;
    outkinect = new unsigned char[3 * 2];

    this->convert32BitTo3Channel8bit(inputKinect, 2, outkinect);
    for (size_t i = 0; i < 2; i++)
    {
        cout << "32bit para 3C8bit: " << (int)outkinect[i * 3 + 1] << "," << (int)outkinect[i * 3] << "," << (int)outkinect[i * 3 + 2] << endl;
    }


    unsigned long int* reversedInKinect;
    reversedInKinect = new unsigned long int[2];
    // convert into single 32bit channel
    this->convert3Channel8bitTo32bit(outkinect, 2, reversedInKinect);
    cout << "Reveted 32 bits: " << reversedInKinect[0] << "," << reversedInKinect[1] << endl;

}


void ofApp::draw()
{

    //depthOFImage.draw(0, 0);
    if (!showPointCloud && currentKinect < texRGB.size())
    {
        drawTextureAtRowAndColumn("RGB Pixels",
            texRGB[currentKinect],
            0, 0);

        drawTextureAtRowAndColumn("RGB Pixels, Registered",
            texRGBRegistered[currentKinect],
            1, 0);

        drawTextureAtRowAndColumn("Depth Pixels, Mapped",
            texDepth[currentKinect],
            1, 1);

        drawTextureAtRowAndColumn("IR Pixels, Mapped",
            texIR[currentKinect],
            0, 1);
    }
    else
    {
        cam.begin();
        ofPushMatrix();
        ofScale(100, -100, -100);
        pointCloud.draw();
        ofPopMatrix();
        cam.end();
    }

    if (kinects.size() < 1) {
        ofDrawBitmapStringHighlight("No Kinects Detected", 40, 40);
    }

    panel.draw();
}

//
//void ofApp::keyPressed(int key)
//{
//    if (key == ' ')
//    {
//        currentKinect = (currentKinect + 1) % kinects.size();
//    }
//    else if (key == 'p')
//    {
//        showPointCloud = !showPointCloud;
//    }
//}


void ofApp::drawTextureAtRowAndColumn(const std::string& title,
    const ofTexture& tex,
    int row,
    int column)
{
    float displayWidth = ofGetWidth() / numColumns;
    float displayHeight = ofGetHeight() / numRows;

    ofRectangle targetRectangle(row * displayWidth,
        column * displayHeight,
        displayWidth,
        displayHeight);

    ofNoFill();
    ofSetColor(ofColor::gray);
    ofDrawRectangle(targetRectangle);

    ofFill();
    ofSetColor(255);
    if (tex.isAllocated())
    {
        ofRectangle textureRectangle(0, 0, tex.getWidth(), tex.getHeight());

        // Scale the texture rectangle to its new location and size.
        textureRectangle.scaleTo(targetRectangle);
        tex.draw(textureRectangle);
    }
    else
    {
        ofDrawBitmapStringHighlight("...",
            targetRectangle.getCenter().x,
            targetRectangle.getCenter().y);
    }

    ofDrawBitmapStringHighlight(title,
        targetRectangle.getPosition() + glm::vec3(14, 20, 0));
}