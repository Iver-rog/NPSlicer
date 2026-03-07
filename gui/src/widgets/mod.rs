use super::Message;

use iced::widget::{Button ,column, row, button, container, text_input, text};
use iced::{Element, Background, Border, Color};

use std::fmt;

pub fn pill_button<'a, Message>(content: impl Into<Element<'a, Message>>) -> Button<'a, Message>
where 
    Message: Clone
{
    button(content)
        .padding([4, 20]) // vertical, horizontal
        .style(|theme, status| {
            use iced::widget::button;

            let palette = theme.extended_palette();

            let base = button::Style {
                background: Some(Background::Color(
                    palette.primary.base.color
                )),
                text_color: palette.primary.base.text,
                border: Border {
                    radius: 999.0.into(), // pill shape
                    width: 0.0,
                    color: Color::TRANSPARENT,
                },
                shadow: Default::default(),
                snap: false,
            };

            match status {
                button::Status::Hovered => button::Style {
                    background: Some(Background::Color(
                        palette.primary.strong.color
                    )),
                    ..base
                },
                button::Status::Pressed => button::Style {
                    background: Some(Background::Color(
                        palette.primary.weak.color
                    )),
                    ..base
                },
                _ => base,
            }
        })
}
#[derive(Debug, Clone)]
pub struct NumberInput<T>{
    pub text: String,
    pub value: T,
}
impl<T> NumberInput<T>
    where T: std::str::FromStr + Copy + fmt::Display,
{
    pub fn new(value:T) -> Self{
        Self{
            text:format!("{value}"),
            value
        }
    }
    pub fn commit(&mut self){
        if let Ok(value) = self.text.parse::<T>(){
            self.value = value;
        } else {
            self.text = format!("{}",self.value);
        }
    }
    pub fn update(&mut self, input: String){
        if let Ok(value) = input.parse::<T>(){
            self.value = value;
            self.text = input;
        } else if input.is_empty() {
            self.text = input;
        }
    }
}

pub fn number_input<'a,T,Message>(
    state: &'a NumberInput<T>,
    on_input: impl Fn(String) -> Message + 'a,
    on_submit: Message,
)-> Element<'a, Message>
// )->TextInput<'a, Message, Theme, Renderer>
where
    T: std::str::FromStr + Copy + 'a,
    Message: 'a + Clone,
{
    let input = text_input("enter a value", &state.text)
        .on_input(on_input)
        .on_submit(on_submit)
        .padding(8);
    input.into()
}

pub fn card<'a,H,C>(header: H, content: C)-> iced::widget::Container<'a, Message>
where
    H: Into<Element<'a,Message>>,
    C: Into<Element<'a,Message>>
{
    container(
    column![
        container(header.into()).padding(iced::Padding{top:4.0,bottom:3.0,left:8.0,right:8.0})
        .style(|theme:&iced::Theme|{
            iced::widget::container::Style{
                background: Some(theme.extended_palette().background.neutral.color.into()),
                border: Border {
                    color: theme.extended_palette().background.weak.color.into(),
                    width: 0.0,
                    radius: iced::border::Radius{
                        top_left: 5.0,
                        top_right: 5.0,
                        bottom_left: 0.0,
                        bottom_right: 0.0,
                    }},
                ..Default::default()
                }
            }).height(iced::Shrink),
        container(content.into()).padding(8)
        .style(|theme:&iced::Theme|{
            iced::widget::container::Style{
                background: Some(theme.extended_palette().background.weakest.color.into()),
                border: Border {
                    color: theme.extended_palette().background.weak.color.into(),
                    width: 1.0,
                    radius: iced::border::Radius{
                        top_left: 0.0,
                        top_right: 0.0,
                        bottom_left: 5.0,
                        bottom_right: 5.0,
                    }},
                ..Default::default()
                }
            })
    ]
    )
}

pub fn text_button<'a>(label: &'a str) -> Button<'a,Message> {
    button(text(label))
        .style(|theme:&iced::Theme, status|{
            let palette = theme.extended_palette();
            let base = button::Style{
                background: None,
                text_color: palette.secondary.base.color,
                border: Default::default(),
                shadow: Default::default(),
                snap: false
            };
            match status{
                button::Status::Hovered => button::Style{
                    text_color: palette.secondary.strong.color,
                    ..base
                },
                button::Status::Pressed => button::Style{
                    text_color: palette.secondary.weak.color,
                    ..base
                },
                _ => base,
            }
    })
}

