
use npslicer_core::{self,async_slice};

mod scene;
use scene::Scene;

mod io;
use io::{pick_file,load_stl};

use std::io::ErrorKind;
use std::path::PathBuf;

use wgpu;
use iced::time::Instant;
use iced::widget::{checkbox, column, row, shader, text, button, container,  horizontal_space, pick_list};
use iced::window;
use iced::{Color, Element, Fill, Subscription};
use iced::task::Task;
use iced::alignment::Horizontal::Right;


fn main() -> iced::Result {
    iced::application(Controls::default, Controls::update, Controls::view)
        .subscription(Controls::subscription)
        .run()
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Printer{
    PrusaMK3SPluss,
    Other(String),
}
impl std::fmt::Display for Printer{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>)-> Result<(),std::fmt::Error>{
        match self{
            Self::PrusaMK3SPluss => write!(f,"PrusaMK3S+"),
            Self::Other(printer_name) => write!(f,"{printer_name}"),
        }
    }
}
impl Default for Printer {
    fn default() -> Self {
        Self::PrusaMK3SPluss
    }
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Filament{
    PLA,
    Other(String),
}
impl std::fmt::Display for Filament{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>)-> Result<(),std::fmt::Error>{
        match self{
            Self::PLA => write!(f,"PLA"),
            Self::Other(filament_name) => write!(f,"{filament_name}"),
        }
    }
}
impl Default for Filament {
    fn default() -> Self {
        Self::PLA
    }
}
pub struct Parameters {
    overhang_angle: usize,
    brim: usize,
    nr_of_perimeters: usize,
    layer_height: f32,
    infill_percentage: usize,
}
impl Default for Parameters{
    fn default() -> Self {
        Self{
            overhang_angle: 20,
            brim: 0,
            nr_of_perimeters: 2,
            layer_height: 0.4,
            infill_percentage: 20,
        }
    }
}

struct Controls {
    start: Instant,
    scene: Scene,

    inputstl: Option<PathBuf>,
    printers: Option<Printer>,
    filament: Option<Filament>,
    parameters: Parameters,
}

#[derive(Debug, Clone)]
enum Message {
    Err(Error),
    Camera(scene::camera::CameraEvent),
    CubeAmountChanged(u32),
    CubeSizeChanged(f32),
    Tick(Instant),
    ShowDepthBuffer(bool),
    LightColorChanged(Color),
    // my stuff
    PrinterChanged(Printer),
    FilamentChanged(Filament),
    SliceModel,
    SlicingComplete(()),
    PickFile,
    STLFilePicked(PathBuf),
}

impl Controls {
    fn new() -> Self {
        Self {
            // ice cubes
            start: Instant::now(),
            scene: Scene::new(),
            // my controls
            printers: Some(Printer::default()),
            filament: Some(Filament::default()),
            parameters: Parameters::default(),
            inputstl: None,
        }
    }

    fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::Err(error) => { println!("{error:?}"); Task::none() },
            Message::CubeAmountChanged(amount) => {
                self.scene.change_amount(amount);
                Task::none()
            }
            Message::CubeSizeChanged(size) => {
                self.scene.size = size;
                Task::none()
            }
            Message::Tick(time) => {
                self.scene.update(time - self.start);
                Task::none()
            }
            Message::ShowDepthBuffer(show) => {
                self.scene.show_depth_buffer = show;
                Task::none()
            }
            Message::LightColorChanged(color) => {
                self.scene.light_color = color;
                Task::none()
            }
            Message::PrinterChanged(printer)     => { self.printers = Some(printer); Task::none()}
            Message::FilamentChanged(filament)   => { self.filament = Some(filament); Task::none()}
            Message::SlicingComplete(result)     => { println!("yay"); Task::none() }
            Message::STLFilePicked(path) => { 
                let file = std::fs::File::open(&path).unwrap();
                let mut reader = std::io::BufReader::new(file);
                self.inputstl = Some(path); 
                self.scene.printbed = stl_io::read_stl(&mut reader).unwrap();
                Task::none() 
            }
            Message::PickFile => { 
                Task::perform( pick_file(),
                    |result| match result {
                        Ok(path) => {
                            match path.extension()
                                .and_then(|ext| ext.to_str())
                                .map(|ext| ext.to_lowercase())
                                .as_deref()
                            {
                                Some("stl") => Message::STLFilePicked(path),
                                Some(ext) => panic!("Unsupported file extension: {ext:?}"),
                                None => panic!("No file extension found: {path:?}"),
                            }
                        }
                        Err(error) => Message::Err(error),
                    }
                )
            },
            Message::SliceModel => { 
                match &self.inputstl {
                    Some(path) => {
                        let settings = npslicer_core::Settings::default();
                        Task::perform( async_slice(path.clone(),settings), Message::SlicingComplete )
                    },
                    None => Task::none()
                }
            }
            Message::Camera(camera_event) => {
                self.scene.camera.handle_event(camera_event);
                Task::none()
            }
        }
    }

    fn view(&self) -> Element<'_, Message> {


        let printers = [
            Printer::PrusaMK3SPluss,
            Printer::Other("halla".into()),
            Printer::Other("yeah dude".into()),
        ];

        let filaments = [
            Filament::PLA,
            Filament::Other("halla mr kis".into()),
            Filament::Other("jalal ".into()),
        ];

        let task_bar = container(row![
            button("file").on_press(Message::PickFile),
            button("settings"),
            horizontal_space(),
            button("slice").on_press(Message::SliceModel),
            button("export G-code file"),
        ]
        .spacing(10)
        )
        .style(container::bordered_box);

        // let printers = row![text(" Printer"), horizontal_rule(30)];
        let printer = container(row![
            text("Printer"),
            horizontal_space(),
            button("edit").padding(0),
        ]).style(container::bordered_box);

        let filament = container(row![
            text("Filament"),
            horizontal_space(),
            button("edit").padding(0),
        ]).style(container::bordered_box);

        let process = container(row![
            text("Process"),
            horizontal_space(),
            button("edit").padding(0),
        ]).style(container::bordered_box);

        let side_menu = container(column![
            printer,
            pick_list(printers,self.printers.clone(),Message::PrinterChanged),
            filament,
            pick_list(filaments,self.filament.clone(),Message::FilamentChanged),
            process,
            control("overhang angle",
                checkbox("", self.scene.show_depth_buffer)
                    .on_toggle(Message::ShowDepthBuffer)
            ),
        ].spacing(5)
        .align_x(Right))
        .width(280)
        .padding(5)
        // .style(container::rounded_box);
        .style(container::bordered_box);

        let shader = shader(&self.scene).width(Fill).height(Fill);

        container(
            column![
                task_bar.width(Fill),
                row![
                    side_menu.height(Fill),
                    shader,
                ],
            ].padding(0),
        ).padding(0)
        .align_top(Fill)
        .into()
    }

    fn subscription(&self) -> Subscription<Message> {
        window::frames().map(Message::Tick)
    }
}

impl Default for Controls {
    fn default() -> Self {
        Self::new()
    }
}

fn control<'a>(
    label: &'static str,
    control: impl Into<Element<'a, Message>>,
) -> Element<'a, Message> {
    row![text(label), control.into()].spacing(10).into()
}


#[derive(Debug, Clone)]
enum Error {
    DialogClosed,
    IO(ErrorKind),
}
