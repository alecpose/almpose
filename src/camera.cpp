#include "../include/camera.hpp"

Camera::Camera(Utils::Options::CameraSetup *setup) {
    if (setup->is_recording){
        camera_object = Metavision::Camera::from_file(setup->file_path);
    }
    else{
        camera_object = Metavision::Camera::from_first_available();
        biases = &camera_object.biases();
        biases->set_from_file(setup->biases_file);
        camera_object.get_device().get_facility<Metavision::I_HW_Register>()->write_register(0xB028, 0x100);
    }
    width = camera_object.geometry().width();
    height = camera_object.geometry().height();

    camera_matrix_eigen = setup->camera_matrix_eigen;
    camera_matrix_cv = setup->camera_matrix_cv;
    dist_coeffs = setup->dist_coeffs;
}

void Camera::initialize_camera() {
    camera_object.cd().add_callback([this](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end)
                          { reader.readEvents(ev_begin, ev_end); });
}
