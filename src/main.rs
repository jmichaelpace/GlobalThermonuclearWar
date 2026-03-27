mod app;
mod map;
mod rendering;
mod scenario;
mod simulation;

use app::App;
use eframe::egui;

fn main() -> eframe::Result<()> {
    // Load environment variables from .env file (if it exists)
    let _ = dotenvy::dotenv();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_min_inner_size([800.0, 600.0])
            .with_title("Global Thermonuclear War"),
        ..Default::default()
    };

    eframe::run_native(
        "Global Thermonuclear War",
        options,
        Box::new(|cc| Ok(Box::new(App::new(cc)))),
    )
}
