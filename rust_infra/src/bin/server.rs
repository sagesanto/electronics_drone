use actix_web::{web, App, HttpServer, HttpResponse};
use std::sync::Mutex;
use std::io;
use matrix_math::Orientation;
// another rust program will push orientation data to this server, and a python program will pull
// orientation will represent 3 3d vectors, one for each axis

struct AppState {
    orientation: Mutex<Orientation>,
}

async fn set_orientation(data: web::Json<Orientation>, app_data: web::Data<AppState>) -> String {
    let mut orientation = app_data.orientation.lock().unwrap();
    *orientation = data.into_inner();
    "Orientation set".to_string()
}

async fn get_orientation(app_data: web::Data<AppState>) -> HttpResponse {
    let orientation = app_data.orientation.lock().unwrap();
    HttpResponse::Ok().json(orientation.clone())
}

#[actix_web::main]
async fn main() -> io::Result<()> {
    let shared_orientation = web::Data::new(AppState {
        orientation: Mutex::new(Orientation { x: [0.0, 0.0, 0.0], y: [0.0, 0.0, 0.0], z: [0.0, 0.0, 0.0] }),
    });

    HttpServer::new(move || {
        App::new()
            .app_data(shared_orientation.clone())
            .route("/orientation", web::get().to(get_orientation))
            .route("/orientation", web::post().to(set_orientation))
    })
    .bind("127.0.0.1:8080")?
    .run()
    .await
}
