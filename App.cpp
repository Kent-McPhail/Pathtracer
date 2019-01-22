/** \file App.cpp */
#include "App.h"
#include "RayTracer.h"

// Tells C++ to invoke command-line main() function even on OS X and Win32.
G3D_START_AT_MAIN();

int main(int argc, const char* argv[]) {
    initGLG3D(G3DSpecification());
    GApp::Settings settings(argc, argv);

    // Change the window and other startup parameters by modifying the
    // settings class.  For example:
    settings.window.caption             = argv[0];

    // Set enable to catch more OpenGL errors
    // settings.window.debugContext     = true;

    // Some common resolutions:
	// settings.window.width = 10; settings.window.height = 10;
    settings.window.width            =  854; settings.window.height       = 480;
    // settings.window.width            = 1024; settings.window.height       = 768;
    //settings.window.width               = 1280; settings.window.height       = 720;
    //settings.window.width             = 1920; settings.window.height       = 1080;
    //settings.window.width            = OSWindow::primaryDisplayWindowSize().x; settings.window.height = OSWindow::primaryDisplayWindowSize().y;
    settings.window.fullScreen          = false;
    settings.window.resizable           = ! settings.window.fullScreen;
    settings.window.framed              = ! settings.window.fullScreen;
    settings.window.defaultIconFilename = "icon.png";

    // Set to true for a significant performance boost if your app can't render at 60fps, or if
    // you *want* to render faster than the display.
    settings.window.asynchronous        = false;

    settings.hdrFramebuffer.depthGuardBandThickness = Vector2int16(64, 64);
    settings.hdrFramebuffer.colorGuardBandThickness = Vector2int16(0, 0);
    settings.dataDir                    = FileSystem::currentDirectory();
    settings.screenCapture.outputDirectory = "../journal/";
    settings.screenCapture.includeAppRevision = false;
    settings.screenCapture.includeG3DRevision = false;
    settings.screenCapture.filenamePrefix = "_";

    settings.renderer.deferredShading = true;
    settings.renderer.orderIndependentTransparency = true;

    return App(settings).run();
}


App::App(const GApp::Settings& settings) : GApp(settings) {
}


// Called before the application loop begins.  Load data here and
// not in the constructor so that common exceptions will be
// automatically caught.
void App::onInit() {
    GApp::onInit();

    setFrameDuration(1.0f / 60.0f);

    // Call setScene(shared_ptr<Scene>()) or setScene(MyScene::create()) to replace
    // the default scene here.
    
    showRenderingStats      = false;

    makeGUI();
    // For higher-quality screenshots:
    // developerWindow->videoRecordDialog->setScreenShotFormat("PNG");
    // developerWindow->videoRecordDialog->setCaptureGui(false);

    loadScene(
#       ifndef G3D_DEBUG
            //"G3D Simple Cornell Box"
			//"G3D Debug Triangle"
			//"G3D Simple Cornell Box (Empty CO)"
			//"G3D Simple Cornell Box (Area Light)"
			//"G3D Simple Cornell Box"
			"G3D Simple Cornell Box (Mirror)"
			//"G3D Sponza"
			//"G3D Sibernik (Statue)"
			//"G3D Simple Cornell Box (Spheres)"
			//"G3D Sponza (Area Light)"
			//"G3D Debug Teapot"
#       else
            //"G3D Simple Cornell Box" // Load something simple
			//"G3D Debug Triangle"
			//"G3D Simple Cornell Box (Empty CO)"
			//"G3D Simple Cornell Box (Area Light)"
			"G3D Simple Cornell Box (Mirror)"
			//"G3D Simple Cornell Box"
			//"G3D Sibernik (Statue)"
			//"G3D Sponza"
			//"G3D Simple Cornell Box (Spheres)"
			//"G3D Sponza (Area Light)"
			//"G3D Debug Teapot"
#       endif
        //developerWindow->sceneEditorWindow->selectedSceneName()  // Load the first scene encountered 
        );
}


void App::makeGUI() {
    debugWindow->setVisible(true);
    developerWindow->videoRecordDialog->setEnabled(true);
	
    // More examples of debugging GUI controls:
    // debugPane->addCheckBox("Use explicit checking", &explicitCheck);
    //debugPane->addTextBox("Rays Per Pixel", &myName);
    //debugPane->addNumberBox("Rays Per Pixel", &raysPerPixel, "rays", GuiTheme::NO_SLIDER, 1, 2048, 1);    
     debugPane->addButton("Render", [this](){ onRender(); });
	

    debugWindow->pack();
    debugWindow->setRect(Rect2D::xywh(0, 0, (float)window()->width(), debugWindow->rect().height()));
}


// This default implementation is a direct copy of GApp::onGraphics3D to make it easy
// for you to modify. If you aren't changing the hardware rendering strategy, you can
// delete this override entirely.
void App::onGraphics3D(RenderDevice* rd, Array<shared_ptr<Surface> >& allSurfaces) {
    if (! scene()) {
        if ((submitToDisplayMode() == SubmitToDisplayMode::MAXIMIZE_THROUGHPUT) && (!rd->swapBuffersAutomatically())) {
            swapBuffers();
        }
        rd->clear();
        rd->pushState(); {
            rd->setProjectionAndCameraMatrix(activeCamera()->projection(), activeCamera()->frame());
            drawDebugShapes();
        } rd->popState();
        return;
    }
	if (m_result) {
		rd->push2D(); {
			Draw::rect2D(rd->viewport(), rd, Color3::white(), m_result);
		} rd->pop2D();
	}

    GBuffer::Specification gbufferSpec = m_gbufferSpecification;
    extendGBufferSpecification(gbufferSpec);
    m_gbuffer->setSpecification(gbufferSpec);
    m_gbuffer->resize(m_framebuffer->width(), m_framebuffer->height());
    m_gbuffer->prepare(rd, activeCamera(), 0, -(float)previousSimTimeStep(), m_settings.hdrFramebuffer.depthGuardBandThickness, m_settings.hdrFramebuffer.colorGuardBandThickness);

    m_renderer->render(rd, activeCamera(), m_framebuffer, scene()->lightingEnvironment().ambientOcclusionSettings.enabled ? m_depthPeelFramebuffer : shared_ptr<Framebuffer>(),
        scene()->lightingEnvironment(), m_gbuffer, allSurfaces);

    // Debugging visualizations and post-process effects
    rd->pushState(m_framebuffer); {
        rd->setProjectionAndCameraMatrix(activeCamera()->projection(), activeCamera()->frame());

        // Call to make the App show the output of debugDraw(...)
        drawDebugShapes();
        const shared_ptr<Entity>& selectedEntity = (notNull(developerWindow) && notNull(developerWindow->sceneEditorWindow)) ? developerWindow->sceneEditorWindow->selectedEntity() : shared_ptr<Entity>();
        scene()->visualize(rd, selectedEntity, allSurfaces, sceneVisualizationSettings(), activeCamera());

        // Post-processed special effects
        m_depthOfField->apply(rd, m_framebuffer->texture(0), m_framebuffer->texture(Framebuffer::DEPTH), activeCamera(), m_settings.hdrFramebuffer.depthGuardBandThickness - m_settings.hdrFramebuffer.colorGuardBandThickness);

        m_motionBlur->apply(rd, m_framebuffer->texture(0), m_gbuffer->texture(GBuffer::Field::SS_POSITION_CHANGE),
            m_framebuffer->texture(Framebuffer::DEPTH), activeCamera(),
            m_settings.hdrFramebuffer.depthGuardBandThickness - m_settings.hdrFramebuffer.colorGuardBandThickness);
    } rd->popState();

    // We're about to render to the actual back buffer, so swap the buffers now.
    // This call also allows the screenshot and video recording to capture the
    // previous frame just before it is displayed.
    if (submitToDisplayMode() == SubmitToDisplayMode::MAXIMIZE_THROUGHPUT) {
        swapBuffers();
    }

    // Clear the entire screen (needed even though we'll render over it, since
    // AFR uses clear() to detect that the buffer is not re-used.)
    rd->clear();
	
    // Perform gamma correction, bloom, and SSAA, and write to the native window frame buffer
    m_film->exposeAndRender(rd, activeCamera()->filmSettings(), m_framebuffer->texture(0), settings().hdrFramebuffer.colorGuardBandThickness.x + settings().hdrFramebuffer.depthGuardBandThickness.x, settings().hdrFramebuffer.depthGuardBandThickness.x, 
        Texture::opaqueBlackIfNull(notNull(m_gbuffer) ? m_gbuffer->texture(GBuffer::Field::SS_POSITION_CHANGE) : nullptr),
        activeCamera()->jitterMotion());
}


void App::onAI() {
    GApp::onAI();
    // Add non-simulation game logic and AI code here
}


void App::onNetwork() {
    GApp::onNetwork();
    // Poll net messages here
}


void App::onSimulation(RealTime rdt, SimTime sdt, SimTime idt) {
    GApp::onSimulation(rdt, sdt, idt);

    // Example GUI dynamic layout code.  Resize the debugWindow to fill
    // the screen horizontally.
    debugWindow->setRect(Rect2D::xywh(0, 0, (float)window()->width(), debugWindow->rect().height()));
}


bool App::onEvent(const GEvent& event) {
    // Handle super-class events
    if (GApp::onEvent(event)) { return true; }

    // If you need to track individual UI events, manage them here.
    // Return true if you want to prevent other parts of the system
    // from observing this specific event.
    //
    // For example,
    // if ((event.type == GEventType::GUI_ACTION) && (event.gui.control == m_button)) { ... return true; }
    // if ((event.type == GEventType::KEY_DOWN) && (event.key.keysym.sym == GKey::TAB)) { ... return true; }
    // if ((event.type == GEventType::KEY_DOWN) && (event.key.keysym.sym == 'p')) { ... return true; }

    if ((event.type == GEventType::KEY_DOWN) && (event.key.keysym.sym == 'p')) { 
        shared_ptr<DefaultRenderer> r = dynamic_pointer_cast<DefaultRenderer>(m_renderer);
        r->setDeferredShading(! r->deferredShading());
        return true; 
    }

    return false;
}


void App::onUserInput(UserInput* ui) {
    GApp::onUserInput(ui);
    (void)ui;
    // Add key handling here based on the keys currently held or
    // ones that changed in the last frame.
}


void App::onPose(Array<shared_ptr<Surface> >& surface, Array<shared_ptr<Surface2D> >& surface2D) {
    GApp::onPose(surface, surface2D);
    // Append any models to the arrays that you want to later be rendered by onGraphics()
}

void App::onRender() {
	const int width = int(window()->width());
	const int height = int(window()->height());
	m_currentImage = Image3::createEmpty(width, height);
	RayTracer raytrace;									 
	//setActiveCamera(debugCamera());	
	Stopwatch timer;
	scene()->onPose(m_surfaceArray);	//Extracts all surfaces from the scene into an array
	scene()->getTypedEntityArray<Light>(m_LightArray);
	raytrace.TraceImage(m_debugCamera, m_currentImage, m_surfaceArray, raysPerPixel, m_LightArray);
	timer.printElapsedTime("Trace");
	debugPrintf("%f s\n", timer.elapsedTime());
	activeCamera()->filmSettings().setSensitivity(0.25f);
	
	// Post-process
	//Converting from image to texture
	//Image is a CPU bound while texture is GPU bound
	const shared_ptr<PixelTransferBuffer>& ptb = CPUPixelTransferBuffer::fromData(m_currentImage->width(), m_currentImage->height(), ImageFormat::RGB32F(), m_currentImage->getCArray(), 1, 1);
	const shared_ptr<Texture>& src = Texture::fromPixelTransferBuffer("Source", ptb, ImageFormat::RGB32F(), Texture::DIM_2D, false);
	if (m_result) {
		m_result->resize(width, height);
	}
	
	//debugPrintf("%f", activeCamera()->filmSettings().sensitivity());
	m_film->exposeAndRender(renderDevice, activeCamera()->filmSettings(), src, settings().hdrFramebuffer.colorGuardBandThickness.x, settings().hdrFramebuffer.depthGuardBandThickness.x, m_result);
	show(m_result, "Raw Radiance");
}


void App::onGraphics2D(RenderDevice* rd, Array<shared_ptr<Surface2D> >& posed2D) {
    // Render 2D objects like Widgets.  These do not receive tone mapping or gamma correction.
    Surface2D::sortAndRender(rd, posed2D);
	
}

void App::onAfterLoadScene(const Any & any, const String & sceneName) {
	GApp::onAfterLoadScene(any, sceneName);
	
	//scene()->onPose(m_surfaceArray);	//Extracts all surfaces from the scene into an array
	//scene()->getTypedEntityArray<Light>(m_LightArray);
}


void App::onCleanup() {
    // Called after the application loop ends.  Place a majority of cleanup code
    // here instead of in the constructor so that exceptions can be caught.
}
