/*
 * Copyright (c) 2019, Mike Wolfram
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef MODM_ST7735_HPP
#	error	"Don't include this file directly, use 'st7735.hpp' instead!"
#endif

namespace modm
{

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::initialize()
{
	reset();

	{
		BatchHandle h(*this);

		this->writeCommand(Command::SwReset, 150ms);
		this->writeCommand(Command::LeaveSleep, 255ms);

		this->writeCommand(Command::FrameCtrlNormalMode, 0, 0, 0);
		this->writeCommand(Command::FrameCtrlIdleMode, 0x01, 0x2c, 0x2d);
		this->writeCommand(Command::FrameCtrlPartialMode, 0x1, 0x2c, 0x2d, 0x01, 0x2c, 0x2d );
		this->writeCommand(Command::InversionCtrl, 0x07 );
		this->writeCommand(Command::PowerCtrl1, 0xa2, 0x02, 0x44 );
		this->writeCommand(Command::PowerCtrl2, 0xc5 );
		this->writeCommand(Command::PowerCtrl3, 0x0a, 0 );
		this->writeCommand(Command::PowerCtrl4, 0x8a, 0x2a );
		this->writeCommand(Command::PowerCtrl5, 0x8a, 0xee );
		this->writeCommand(Command::VComCtrl1, 0x0e );
		this->writeCommand(Command::InversionOff );
		this->writeCommand(Command::PixelFormatSet, 5 ); //16 bit/pixel
		this->writeCommand(Command::PositiveGammaCorrection, 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
                                                             0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10);
		this->writeCommand(Command::NegativeGammaCorrection, 0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                                                             0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10);

		this->writeCommand(Command::NormalMode, 10ms );
		this->writeCommand(Command::DisplayOn, 100ms );
		setOrientation(orientation);
	}
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::reset(bool hardReset /* = false */)
{
	if (hardReset)
	{
		Reset::set();
		modm::delay_ms(5);
		Reset::reset();
		modm::delay_ms(5);
		Reset::set();
		modm::delay_ms(5);
	}
	else {
		BatchHandle h(*this);
		this->writeCommand(Command::SwReset);
		modm::delay_ms(5);
	}
}

template <class Interface, class Reset, std::size_t BufferSize>
uint16_t
St7735<Interface, Reset, BufferSize>::getIcModel()
{
	BatchHandle h(*this);

	uint8_t buffer[4] { 0 };
	this->readData(Command::ReadID, buffer, 4);
	return (buffer[2] << 8) | buffer[3];
}

template <class Interface, class Reset, std::size_t BufferSize>
inline void
St7735<Interface, Reset, BufferSize>::setOrientation(glcd::Orientation orientation)
{
	using MemoryAccessCtrl_t = st7735::MemoryAccessCtrl_t;
	using MemoryAccessCtrl = st7735::MemoryAccessCtrl;
	MemoryAccessCtrl_t madCtrl { 0 };

	switch (orientation)
	{
		case glcd::Orientation::Portrait90:
			madCtrl |= MemoryAccessCtrl::MV | MemoryAccessCtrl::MX;
			break;
		case glcd::Orientation::Landscape180:
			madCtrl |= MemoryAccessCtrl::MX | MemoryAccessCtrl::MY;
			break;
		case glcd::Orientation::Portrait270:
			madCtrl |= MemoryAccessCtrl::MV | MemoryAccessCtrl::MY;
			break;
		default:
//			madCtrl |= MemoryAccessCtrl::ML;
			break;
	}

	this->orientation = orientation;

	BatchHandle h(*this);
	this->writeCommandValue8(Command::MemoryAccessCtrl, madCtrl.value);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::turnOn()
{
	BatchHandle h(*this);
	this->writeCommand(Command::DisplayOn);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::turnOff()
{
	BatchHandle h(*this);
	this->writeCommand(Command::DisplayOff);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::setIdle(bool enable)
{
	BatchHandle h(*this);
	this->writeCommand(enable ? Command::IdleModeOn : Command::IdleModeOff);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::enableSleep(bool enable)
{
	BatchHandle h(*this);
	this->writeCommand(enable ? Command::EnterSleep : Command::LeaveSleep);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::setInvert(bool invert)
{
	BatchHandle h(*this);
	this->writeCommand(invert ? Command::InversionOn : Command::InversionOff);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::clear()
{
	auto const saveForegroundColor { foregroundColor };
	foregroundColor = backgroundColor;
	fillRectangle(glcd::Point(0, 0), Width, Height);
	foregroundColor = saveForegroundColor;
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::drawHorizontalLine(
		glcd::Point start, uint16_t length)
{
	uint16_t const pixelValue { modm::toBigEndian(foregroundColor.color) };
	auto minLength { std::min(std::size_t(length), BufferSize) };
	uint16_t *buffer16 { reinterpret_cast<uint16_t *>(buffer) };
	std::fill(buffer16, buffer16+minLength, pixelValue);

	BatchHandle h(*this);

	setClipping(start.getX(), start.getY(), length, 1);
	while (length > BufferSize)
	{
		this->writeData(buffer, BufferSize * 2);
		length -= BufferSize;
	}
	this->writeData(buffer, length * 2);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::drawVerticalLine(
		glcd::Point start, uint16_t length)
{
	uint16_t const pixelValue { modm::toBigEndian(foregroundColor.color) };
	auto minLength { std::min(std::size_t(length), BufferSize) };
	uint16_t *buffer16 { reinterpret_cast<uint16_t *>(buffer) };
	std::fill(buffer16, buffer16+minLength, pixelValue);

	BatchHandle h(*this);

	setClipping(start.getX(), start.getY(), 1, length);
	while (length > BufferSize)
	{
		this->writeData(buffer, BufferSize * 2);
		length -= BufferSize;
	}
	this->writeData(buffer, length * 2);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::fillRectangle(
		glcd::Point upperLeft, uint16_t width, uint16_t height)
{
	auto const x { upperLeft.getX() };
	auto const y { upperLeft.getY() };
	std::size_t pixelCount { std::size_t(width) * std::size_t(height) };

	uint16_t const pixelValue { modm::toBigEndian(foregroundColor.color) };
	auto minLength { std::min(std::size_t(pixelCount), BufferSize) };
	uint16_t *buffer16 { reinterpret_cast<uint16_t *>(buffer) };
	std::fill(buffer16, buffer16+minLength, pixelValue);

	BatchHandle h(*this);

	setClipping(x, y, width, height);
	while (pixelCount > BufferSize)
	{
		this->writeData(buffer, BufferSize * 2);
		pixelCount -= BufferSize;
	}
	if (pixelCount)
		this->writeData(buffer, pixelCount * 2);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::fillCircle(
		glcd::Point center, uint16_t radius)
{
	uint8_t const setColor[] { uint8_t((foregroundColor.color >> 8) & 0xff),
			uint8_t(foregroundColor.color & 0xff) };

	int16_t f = 1 - radius;
	int16_t ddF_x = 0;
	int16_t ddF_y = -2 * radius;
	uint16_t x = 0;
	uint16_t y = radius;

	BatchHandle h(*this);

	setClipping(center.getX() - radius, center.getY(), 2 * radius, 1);
	for (std::size_t i = 0; i < 2 * radius; ++i)
		this->writeData(setColor, 2);

	while(x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x + 1;

		setClipping(center.getX() - x, center.getY() - y, 2 * x, 1);
		for (std::size_t i = 0; i < 2 * x; ++i)
			this->writeData(setColor, 2);
		setClipping(center.getX() - y, center.getY() - x, 2 * y, 1);
		for (std::size_t i = 0; i < 2 * y; ++i)
			this->writeData(setColor, 2);
		setClipping(center.getX() - x, center.getY() + y, 2 * x, 1);
		for (std::size_t i = 0; i < 2 * x; ++i)
			this->writeData(setColor, 2);
		setClipping(center.getX() - y, center.getY() + x, 2 * y, 1);
		for (std::size_t i = 0; i < 2 * y; ++i)
			this->writeData(setColor, 2);
	}
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::drawImageRaw(glcd::Point upperLeft,
		uint16_t width, uint16_t height, modm::accessor::Flash<uint8_t> data)
{
	uint8_t const setColor[] { uint8_t((foregroundColor.color >> 8) & 0xff),
			uint8_t(foregroundColor.color & 0xff) };
	uint8_t const clearColor[] { uint8_t((backgroundColor.color >> 8) & 0xff),
			uint8_t(backgroundColor.color & 0xff) };

	BatchHandle h(*this);

	setClipping(upperLeft.getX(), upperLeft.getY(), width, height);

	uint8_t bit = 0x01;
	for (uint16_t r = 0; r < height; ++r)
	{
		for (uint16_t w = 0; w < width; ++w)
		{
			uint8_t byte = data[(r / 8) * width + w];
			if (byte & bit)
				this->writeData(setColor, 2);
			else
				this->writeData(clearColor, 2);
		}
		// TODO: optimize, use ROL (rotate left)
		bit <<= 1;
		if (bit == 0)
			bit = 0x01;
	}
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::drawRaw(glcd::Point upperLeft,
		uint16_t width, uint16_t height, color::Rgb565* data)
{
	BatchHandle h(*this);

	uint16_t* buffer = (uint16_t*)data;
	for(size_t i = 0; i < size_t(width*height); i++) {
		buffer[i] = modm::fromBigEndian(buffer[i]);
	}

	setClipping(upperLeft.getX(), upperLeft.getY(), width, height);
	this->writeData((uint8_t*)buffer, width * height * 2);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::setColoredPixel(
		int16_t x, int16_t y, color::Rgb565 const &color)
{
	auto const pixelColor { color };
	uint8_t const setColor[] { uint8_t((pixelColor.color >> 8) & 0xff), uint8_t(pixelColor.color & 0xff) };

	BatchHandle h(*this);

	this->setClipping(x, y, 1, 1);
	this->writeData(setColor, 2);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::setClipping(
		uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
	uint8_t buffer[4];

	buffer[0] = uint8_t((x >> 8) & 0xff);
	buffer[1] = uint8_t(x & 0xff);
	buffer[2] = uint8_t(((x + width - 1) >> 8) & 0xff);
	buffer[3] = uint8_t((x + width - 1) & 0xff);
	this->writeCommand(Command::ColumnAddressSet, buffer, 4);

	buffer[0] = uint8_t((y >> 8) & 0xff);
	buffer[1] = uint8_t(y & 0xff);
	buffer[2] = uint8_t(((y + height - 1) >> 8) & 0xff);
	buffer[3] = uint8_t((y + height - 1) & 0xff);
	this->writeCommand(Command::PageAddressSet, buffer, 4);
	this->writeCommand(Command::MemoryWrite);
}

template <class Interface, class Reset, std::size_t BufferSize>
void
St7735<Interface, Reset, BufferSize>::drawBitmap(glcd::Point upperLeft,
		uint16_t width, uint16_t height, modm::accessor::Flash<uint8_t> data)
{
	BatchHandle h(*this);

	setClipping(upperLeft.getX(), upperLeft.getY(), width, height);
	for (int i = 0; i < width * height; ++i) {
		buffer[0] = data[i*2+1];
		buffer[1] = data[i*2];
		this->writeData(buffer, 2);
	}
//	this->writeData(data.getPointer(), width * height * 2);
}

} // namespace modm
