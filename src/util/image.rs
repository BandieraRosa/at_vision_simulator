use bevy::prelude::Image;
use r2r::sensor_msgs::msg::CompressedImage;
use r2r::std_msgs::msg::Header;

pub fn compress_image(header: Header, img: &Image) -> CompressedImage {
    let img = img.clone().try_into_dynamic().unwrap().to_rgb8();
    use image::codecs::jpeg::JpegEncoder;
    use image::{ImageBuffer, Rgb};
    use std::io::Cursor;

    let width = img.width();
    let height = img.height();

    let buffer: ImageBuffer<Rgb<u8>, _> =
        ImageBuffer::from_raw(width, height, img.into_raw()).unwrap();

    let mut cursor = Cursor::new(Vec::new());
    let mut encoder = JpegEncoder::new(&mut cursor);
    encoder.encode_image(&buffer).expect("JPEG encode failed");
    let compressed_data = cursor.into_inner();

    CompressedImage {
        header,
        format: "jpeg".to_string(),
        data: compressed_data,
    }
}
