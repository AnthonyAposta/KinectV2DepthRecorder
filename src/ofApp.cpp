#include "ofApp.h"
//NOTE: if you are unable to connect to your device on OS X, try unplugging and replugging in the power, while leaving the USB connected.
//ofxKinectV2 will only work if the NUI Sensor shows up in the Superspeed category of the System Profiler in the USB section.

//On OS X if you are not using the example project. Make sure to add OpenCL.framework to the Link Binary With Library Build Phase 
//and change the line in Project.xcconfig to OTHER_LDFLAGS = $(OF_CORE_LIBS) $(OF_CORE_FRAMEWORKS) -framework OpenCL


void ofApp::setup()
{
    ofSetFrameRate(30);
    ofSetVerticalSync(true);
    ofBackground(100);

    
    cam.setUpAxis(glm::vec3(0, -1, 0));
    cam.setGlobalPosition(glm::vec3(0, 0, -4));
    cam.lookAt(glm::vec3(0, 0, 0), glm::vec3(0, -1, 0));


    // Uncomment for verbose info from libfreenect2
    ofSetLogLevel(OF_LOG_VERBOSE);

    #pragma region KinectConfigs
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

    #pragma endregion

    h = 424;
    w = 512;
    depthFrameRGB = new unsigned char[w * h * 3];
    revertedRawPixelsInt = new unsigned long int[h * w];

    depthOFImage.allocate(w, h, OF_IMAGE_COLOR);
    RGBOFImage.allocate(w, h, OF_IMAGE_COLOR);

    //	Spout - Initialise the Spout sender
    spoutSenderDepth.init("DepthVideo", w, h);// , OF_PIXELS_RGB);
    spoutSenderRGB.init("RGBVideo", w, h);// , OF_PIXELS_RGB);

    // set pointcloud primitive
    pointCloud.setMode(OF_PRIMITIVE_POINTS);

    //// Set GUIs parameters

    // pointcloud gui
    unsigned short maxDistaceBitValue = 13000;
    unsigned short minDistaceBitValue = 11381;

    recordingGui.setup("Near/Far clipping");
    recordingGui.add(minDistanceBits.set("Near", minDistaceBitValue, minDistaceBitValue, maxDistaceBitValue));
    recordingGui.add(maxDistanceBits.set("Far", maxDistaceBitValue, minDistaceBitValue, maxDistaceBitValue));

    // playbakc gui
    playbackGui.setup("Near/Far clipping");
    playbackGui.add(NearMeters.set("Near", 0.5, 0.5, 4.5));
    playbackGui.add(FarMeters.set("Far", 4.5, 0.5, 4.5));

    playbackGui.add(LowMeters.set("Low", 2.5, -2.5, 2.5));
    playbackGui.add(HighMeters.set("High", -2.5, -2.5, 2.5));

    playbackGui.add(LeftMeters.set("Left", -2.5, -2.5, 2.5));
    playbackGui.add(RightMeters.set("Right", 2.5, -2.5, 2.5));
}

// Update methods
void ofApp::update()
{

    if (showDebugScreen) {

        updateFromKinect();
    }
    else if (showPointCloud)
    {
        updateFromKinect();
    }
    else if (showPlaybackScreen)
    {
        if (!onSavingObj)
        {
            updateFromVideoFile();
        }
        else
        {
            updateFromVideoFileOnSaving();
        }
        
    }
}

void ofApp::updateFromKinect() {
    
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

                // Transform
                this->convert32BitTo3Channel8bit(RawPixelsInt, w * h, depthFrameRGB);
                depthOFImage.setFromPixels(depthFrameRGB, w, h, OF_IMAGE_COLOR);
                
                this->convert3Channel8bitTo32bit(depthOFImage.getPixelsRef().getData(), w * h, revertedRawPixelsInt);

                // Revert
                cropDepthData(revertedRawPixelsInt);
                this->convert32BitTo3Channel8bit(revertedRawPixelsInt, w * h, depthFrameRGB);
                depthOFImage.setFromPixels(depthFrameRGB, w, h, OF_IMAGE_COLOR);

                // Get RGB image
                
                RGBImage.loadData(kinects[d]->getRegisteredPixels());
                //RGBOFImage.setFromPixels();
                //RGBOFImage.update();

                spoutSenderDepth.send(depthOFImage.getTexture());
                spoutSenderRGB.send(RGBImage);

                texDepth[d].loadData(depthOFImage.getPixels());
            }

            if (showPointCloud)
            {
                fillVboMesh(revertedRawPixelsInt, kinects[d]->getRegisteredPixels(), h, w);
            }
        }
    }
}

void ofApp::updateFromVideoFile() {

    if (depthVidPlayer.isLoaded()) {

        depthVidPlayer.update();
        unsigned char* videoFrameRGB = depthVidPlayer.getPixelsRef().getPixels();
        depthOFImage.setFromPixels(videoFrameRGB, w * 2, h, OF_IMAGE_COLOR);
        RGBOFImage.setFromPixels(videoFrameRGB, w * 2, h, OF_IMAGE_COLOR);

        depthOFImage.crop(0, 0, w, h);
        RGBOFImage.crop(w, 0, w, h);

        this->convert3Channel8bitTo32bit(depthOFImage.getPixelsRef().getData(), w * h, revertedRawPixelsInt);
        fillVboMesh(revertedRawPixelsInt, RGBOFImage.getPixelsRef(), h, w);
    }

}

void ofApp::updateFromVideoFileOnSaving() {

    depthVidPlayer.setPaused(true);

    // update mesh
    unsigned char* videoFrameRGB = depthVidPlayer.getPixelsRef().getPixels();
    depthOFImage.setFromPixels(videoFrameRGB, w * 2, h, OF_IMAGE_COLOR);
    RGBOFImage.setFromPixels(videoFrameRGB, w * 2, h, OF_IMAGE_COLOR);

    depthOFImage.crop(0, 0, w, h);
    RGBOFImage.crop(w, 0, w, h);

    this->convert3Channel8bitTo32bit(depthOFImage.getPixelsRef().getData(), w * h, revertedRawPixelsInt);
    fillVboMesh(revertedRawPixelsInt, RGBOFImage.getPixelsRef(), h, w);


    // save mesh
    obj_folder_path = SaveObjDialog.filePath;
    obj_filename = obj_folder_path + "\\OBJ_data_" + to_string(depthVidPlayer.getCurrentFrame()) + ".ply";
    pointCloud.save(obj_filename);

    cout << "current frame: " << depthVidPlayer.getCurrentFrame() << " total: " << depthVidPlayer.getTotalNumFrames() << endl;
    depthVidPlayer.setPaused(false);
    depthVidPlayer.nextFrame();
    depthVidPlayer.update();

    if (depthVidPlayer.getCurrentFrame() == depthVidPlayer.getTotalNumFrames()) {
        onSavingObj = false;
    }

}

// Methods to make data convertsions and crops
void ofApp::convert32BitTo3Channel8bit(unsigned long int* in, int in_size, unsigned char* out)
{
    // Input has 8 bytes, only last two ar used.
    // The output order will be (2th byte, 3th byte, 0)
 
    for (int i = 0; i < in_size; i++) {
        
        out[i * 3 + 0] = static_cast<unsigned char>(((in[i]>>16) & 0xFF)); // lower byte
        // 0 to 69
        out[i * 3 + 1] = static_cast<unsigned char>((in[i] >> 24) & 0xFF); // higher byte
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

void ofApp::cropDepthData(unsigned long int* depthData) {
    int size = h * w;
    unsigned long int minVal = 1e5 * (unsigned long int)(minDistanceBits.get());
    unsigned long int maxVal = 1e5 * (unsigned long int)(maxDistanceBits.get());


    for (size_t i = 0; i < size; i++)
    {
        //cout << depthData[i] << "<" << minDistance.get() << ":::" << depthData[i] << ">" << maxDistance.get() << endl;
        if ((depthData[i] < minVal) || (depthData[i] > maxVal)) {
            depthData[i] = 0;
        }
    }
}


// Methods to handle points and the mesh
void ofApp::getPointXYZ(unsigned long int* frame, int r, int c, float& x, float& y, float& z)
{
    // cx: 257.244507, cy: 208.579605, fx: 365.829010, fy: 365.829010

    const float cx = 257.244507;
    const float cy = 208.579605;
    const float fx = 1 / 365.829010;
    const float fy = 1 / 365.829010;

    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    //float* undistorted_data = (float*)frame;
    float* undistorted_data = reinterpret_cast<float*>(frame);

    // 512 não deveria ser hardcoded aqui
    const float depth_val = undistorted_data[512 * r + c] / 1000.0f; //scaling factor, so that value of 1 is one meter.
    if (isnan(depth_val) || depth_val <= 0.001)
    {
        //depth value is not valid
        x = y = z = bad_point;
    }
    else
    {
        x = (c + 0.5 - cx) * fx * depth_val;
        y = (r + 0.5 - cy) * fy * depth_val;
        z = depth_val;
    }
}

void ofApp::fillVboMesh(unsigned long int* depthFrame, ofPixels rgbRegiteredPixels, int h, int w) {
   
    pointCloud.clear();
    for (std::size_t x = 0; x < w; x++)
    {
        for (std::size_t y = 0; y < h; y++)
        {

            glm::vec3 position = { 0,0,0 };
            getPointXYZ(depthFrame, y, x, position.x, position.y, position.z);
            position.x = -position.x;
            
            // if in playbackmode um use the far/near crop
            if (showPlaybackScreen) {

                if ((position.z > NearMeters.get()) && (position.z < FarMeters.get())) {
                    if ((position.x > LeftMeters.get()) && (position.x < RightMeters.get())) {
                        if ((position.y > HighMeters.get()) && (position.y < LowMeters.get())) {
                    
                            pointCloud.addVertex(position);
                            pointCloud.addColor(rgbRegiteredPixels.getColor(x, y));

                        }
                    }
                }
            }
            else {
                pointCloud.addVertex(position);
                pointCloud.addColor(rgbRegiteredPixels.getColor(x, y));
            }

            
        }
    }
}

// Methods to hadle keys presses
void ofApp::keyPressed(int key)
{
    if (key == 'q')
    {
        showDebugScreen = true;
        showPointCloud = false;
        showPlaybackScreen = false;
    }
    else if (key == 'w')
    {
        showDebugScreen = false;
        showPointCloud = true;
        showPlaybackScreen = false;

    }
    else if (key == 'e')
    {
        showDebugScreen = false;
        showPointCloud = false;
        showPlaybackScreen = true;
        pointCloud.clear();
    }
    else if (key == 'r')
    {
        if (showPointCloud == true) {
            onRecording = true;
        }
    }
    else if (key == 'l')
    {
        if (showPlaybackScreen)
        {
            dialogResult = ofSystemLoadDialog("Load a depth video file");
            depthVidPlayer.load(dialogResult.filePath);
            depthVidPlayer.play();
            depthVidPlayer.firstFrame();
        }
    }
    else if (key == 's') {
        if (showPlaybackScreen && depthVidPlayer.isLoaded()) {
            
            SaveObjDialog = ofSystemLoadDialog("Load a depth video file", true);
            if (SaveObjDialog.bSuccess) {
                depthVidPlayer.firstFrame();
                //depthVidPlayer.stop();
                onSavingObj = true;
            }
        }
    }
    else if (key == 'c')
    {
        cam.setUpAxis(glm::vec3(0, -1, 0));
        cam.setGlobalPosition(glm::vec3(0, 0, -400));
        cam.lookAt(glm::vec3(0, 0, 0), glm::vec3(0, -1, 0));
    }
    else if (key == ofKey::OF_KEY_RIGHT) {
        if (showPlaybackScreen and depthVidPlayer.isLoaded())
        {
            int frame = depthVidPlayer.getCurrentFrame();
            depthVidPlayer.setFrame(frame + 60);
        }
    }
    else if (key == ofKey::OF_KEY_LEFT) {
        if (showPlaybackScreen and depthVidPlayer.isLoaded())
        {
            int frame = depthVidPlayer.getCurrentFrame();
            depthVidPlayer.setFrame(frame - 60);
        }
    }

}

// Drawing methods
void ofApp::drawDebugScren() {
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

void ofApp::drawPointCloud() {

    float boxDepth = 5;
    float boxHeight = 5;
    float boxWidth = 5;

    float boxPosZ = 0;
    float boxPosY = 0;
    float boxPosX = 0;

    if (showPointCloud) {
        boxDepth = 4.5 - 0.5;
        boxPosZ = (boxDepth / 2) + 0.5;
    }
    else if (showPlaybackScreen) {
        boxDepth = abs(FarMeters.get() - NearMeters.get());
        boxWidth = abs( LeftMeters.get() - RightMeters.get() );
        boxHeight = abs( LowMeters.get() - HighMeters.get() );

        
        boxPosZ = (boxDepth / 2) + NearMeters.get();
        boxPosX = (boxWidth / 2) + LeftMeters.get();
        boxPosY = (boxHeight/ 2) + HighMeters.get();


    }


    cam.begin();

    ofPushMatrix();
    ofScale(100, 100, 100);
    
    // draw axis
    ofDrawAxis(1);

    // draw grid
    ofRotateZ(-90);
    ofDrawGridPlane(1, 5, true);
    ofPopMatrix();

    ofPushMatrix();
    ofScale(100, 100, 100);

    // draw points
    pointCloud.drawVertices();
    
    // draw referece cube
    ofSetLineWidth(4);
    ofNoFill();
    ofColor(10);
    ofDrawBox(boxPosX, boxPosY, boxPosZ, boxWidth, boxHeight, boxDepth);
    ofFill();
    ofColor(255);
    ofPopMatrix();

    cam.end();
}

void ofApp::draw()
{
    int legendBoxDist = ofGetWidth() - 150;
    if (showDebugScreen && currentKinect < texRGB.size())
    {
        ofBackground(0);
        drawDebugScren();
        recordingGui.draw();
        ofDrawBitmapStringHighlight("Q: Debug mode\nW: Preview mode\nE: Playback mode", legendBoxDist, 20);

    }
    else if (showPointCloud)
    {
        ofBackground(100);
        drawPointCloud();
        recordingGui.draw();
        ofDrawBitmapStringHighlight("Q: Debug mode\nW: Preview mode\nE: Playback mode\nC: Reset cam view", legendBoxDist, 20);


    }
    else if (showPlaybackScreen) {

        ofBackground(100);
        drawPointCloud();
        playbackGui.draw();
        ofDrawBitmapStringHighlight("Q: Debug mode\nW: Preview mode\nE: Playback mode\nL: Load playback\nS: Save OBJ files\nC: Reset cam view\n->: Forward\n<-: Backward  ", legendBoxDist, 20);
    }

    if (kinects.size() < 1) {
        ofDrawBitmapStringHighlight("No Kinects Detected", 40, 40);
    }

  

}

void ofApp::drawTextureAtRowAndColumn(const std::string& title, const ofTexture& tex, int row, int column)
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