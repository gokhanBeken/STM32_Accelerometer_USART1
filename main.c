#include "STM32F4xx.h"

//dasdasd
unsigned char PWM[8]; // PWM registerler
unsigned char SRG[8]; // Shadow Registerler
unsigned char CNTR; // PWM Counter

//bayraklar
char x = 0, y = 0; //koordinat bayraklarimiz

/*****************************************************************************************************
 CPU PLL ile 168Mhz de kosturulur
 AHB frekansy 168 Mhz
 APB1 frekansy 42 Mhz
 APB2 frekansy 84 Mhz
 *****************************************************************************************************/
void SystemInit2() {
	volatile unsigned int i;
	(*((int*) 0xE000ED88)) |= 0x0F00000;
	for (i = 0; i < 0x00100000; i++)
		; // OSC oturtma ve kurtarma rutini
	RCC->CFGR |= 0x00009400; // AHB ve APB hizlarini max degerlere set edelim
	RCC->CR |= 0x00010000; // HSE Xtal osc calismaya baslasin
	while (!(RCC->CR & 0x00020000))
		; // Xtal osc stabil hale gelsin
	RCC->PLLCFGR = 0x07402A04; // PLL katsayilarini M=4, N=168, P=2 ve Q=7 yapalim
	RCC->CR |= 0x01000000; // PLL calismaya baslasin  (Rehber Sayfa 95)
	while (!(RCC->CR & 0x02000000))
		; // Pll hazir oluncaya kadar bekle
	FLASH->ACR = 0x00000605; // Flash ROM icin 5 Wait state secelim ve ART yi aktif edelim (Rehber Sayfa 55)
	RCC->CFGR |= 0x00000002; // Sistem Clk u PLL uzerinden besleyelim
	while ((RCC->CFGR & 0x0000000F) != 0x0000000A)
		; // Besleninceye kadar bekle
	RCC->AHB1ENR |= 0x0000001F; // GPIO A,B,C,D,E clock'u aktif edelim
	GPIOD->MODER = 0x55550000; // GPIOD nin 15, 14, 13, 12, 11, 10, 9, 8 pinleri cikis tanimlandi (LEDler icin)
	GPIOD->OSPEEDR = 0xFFFFFFFF; // GPIOD nin tum cikislari en yuksek hizda kullanacagiz
//  GPIOA da A7, A6, A5 pinleri, LIS302DL cipiyle haberlesmek icin SPI moduna alinacak

	GPIOA->AFR[0] = 0x55500000; // SPI1 secelim (Rehber Sayfa 141), Hard Sayfa 49
	GPIOA->MODER |= 0x0000A800; // A7,A6,A5 Alternatif fonksiyon
	RCC->APB2ENR |= 0x00001000; // SPI1 clk enable   //   GPIOE3 pinini output tanimlayalim (LIS302DL SPI/I2C secimi)
	GPIOE->MODER = 0x00000040; // GPIOE nin 3 nolu pini cikis tanimlandi
	GPIOE->OSPEEDR = 0xFFFFFFFF; // GPIOE nin tum cikislari en yuksek hizda kullanacagiz
	GPIOE->BSRRL = 0x0008; // GPIOE3=1; LIS302DL CS=1
	SPI1->CR1 = 0x00000B7F; // SPI1 16 bit, master , fulldublex vs
	SPI1->CR2 = 0X0000;

	RCC->APB1ENR |= 0x00000020; // Timer7 CLK'u aktif edelim (84 Mhz)
	TIM7->CR1 = 0x0080; // Otomatik Reload
	TIM7->PSC = 839; // Prescaler degerimiz 839, Count frekansimiz = fCK_PSC / (Yuklenen Deger + 1) 84E6 / (840) = 100 KHz
	TIM7->ARR = 1; // Counter, Decimal 1 olunca basa donsun. Her 20 mikrosaniye de bir timer int olusacak.
	TIM7->DIER = 0x0001; // Update Int enable
	NVIC->ISER[1] = 0X00800000; // NVIC de Timer 7 interrupta izin verelim
	TIM7->CR1 |= 0x0001; // Counter Enable

}
void TIM7_IRQHandler() {
	unsigned short d, i, j;
	TIM7->SR = 0; // Timer Int Flagini silelim
	//d = GPIOD->ODR | 0xFF00; //bu satira gerek yok, o pinleri baska bir is icin kullanamayiz yoksa
	d = 0xFF00;
	CNTR++;
	if (!CNTR) {
		for (i = 0; i < 8; i++)
			SRG[i] = PWM[i];
	}
	j = 0x8000;
	for (i = 0; i < 8; i++) {
		if (CNTR >= SRG[i])
			d &= ~j;
		j = j >> 1;
	}
	/*
	 if (((d) & (1 << 12)) - (1 << 12) == 0) { //demekki 12. bit 1 olmus yani yesil led yani led4 yani -x kordinati
	 GPIOD->ODR |= (1 << 12);
	 x = 0;
	 } else {
	 GPIOD->ODR &= ~(1 << 12);
	 //x=1;
	 }

	 if (((d) & (1 << 14)) - (1 << 14) == 0) { //demekki 14. bit 1 olmus yani kirmizi led yani led5 yani +x kordinati
	 GPIOD->ODR |= (1 << 14);
	 x = 1;
	 } else {
	 GPIOD->ODR &= ~(1 << 14);
	 }

	 if (((d) & (1 << 15)) - (1 << 15) == 0) { //demekki 15. bit 1 olmus yani mavi led yani led6 yani -y kordinati
	 GPIOD->ODR |= (1 << 15);
	 y = 0;
	 } else {
	 GPIOD->ODR &= ~(1 << 15);
	 //y=1;
	 }

	 if (((d) & (1 << 13)) - (1 << 13) == 0) { //demekki 13. bit 1 olmus yani turuncu led yani led3 yani +y kordinati
	 GPIOD->ODR |= (1 << 13);
	 y = 1;
	 } else {
	 GPIOD->ODR &= ~(1 << 13);
	 }


	 */

	if (((d) & (1 << 14)) - (1 << 14) == 0) { //demekki 14. bit 1 olmus yani kirmizi led yani led5 yani +x kordinati
		x = 1;
	}
	if (((d) & (1 << 12)) - (1 << 12) == 0) { //demekki 12. bit 1 olmus yani yesil led yani led4 yani -x kordinati
		x = 0;
	}

	if (((d) & (1 << 13)) - (1 << 13) == 0) { //demekki 13. bit 1 olmus yani turuncu led yani led3 yani +y kordinati
		y = 1;
	}
	if (((d) & (1 << 15)) - (1 << 15) == 0) { //demekki 15. bit 1 olmus yani mavi led yani led6 yani -y kordinati
		y = 0;
	}

	if (x) { //demekki 14. bit 1 olmus yani kirmizi led yani led5 yani +x kordinati
		GPIOD->ODR |= (1 << 14);
		GPIOD->ODR &= ~(1 << 12);
	} else {
		GPIOD->ODR |= (1 << 12);
		GPIOD->ODR &= ~(1 << 14);
	}

	if (y) { //demekki 14. bit 1 olmus yani kirmizi led yani led5 yani +x kordinati
		GPIOD->ODR |= (1 << 13);
		GPIOD->ODR &= ~(1 << 15);
	} else {
		GPIOD->ODR |= (1 << 15);
		GPIOD->ODR &= ~(1 << 13);
	}

	//GPIOD->ODR = d;
}
signed char SPI_CMD(short DAT) {
	signed char RxDat;
	GPIOE->BSRRH = 0x0008; // LIS302DL CS=0
	RxDat = SPI1->SR; // AMAC DELAY (kalmasinda fayda var)
	SPI1->DR = DAT; // Komutu yukle
	while (!(SPI1->SR & 0x01))
		; // RX BUF bos ise beklemede kal
	while (SPI1->SR & 0x80)
		; // BSY durumu varsa kalkmasini bekleyelim
	RxDat = SPI1->DR; // Cipten gelen veriyi oku
	while (SPI1->SR != 0x02)
		; // CS=1 yapmadan once cipin orjinal duruma donmeyi bekleyelim
	GPIOE->BSRRL = 0x0008; // LIS302DL CS=1
	return (RxDat);
}
signed char Write(char Adr, unsigned char Data) {
	return (SPI_CMD(((Adr & 0x3F) << 8) | Data));
}
signed char Read(char Adr) {
	return (SPI_CMD(((Adr & 0x3F) | 0x80) << 8));
}

int main() {
	SystemInit2();
	int i;
	signed char x[16]; // PWM registerler
	signed char y[16]; // PWM registerler
	signed char who, xo, yo, b;
	signed short a;

	if (Read(0x0F) == 0x3B) // Who are you ?
			{
		Write(0x20, 0x47); // Data Rate=100Hz, Full Scale=2g, Activate, x,y,z enable
		while (!(Read(0x27) & 1))
			;
		xo = Read(0x29);
		while (!(Read(0x27) & 2))
			;
		yo = Read(0x2B);

		while (1) {
			who = Read(0x27); // Statusu ogrenelim. Kim hazir kim degil?
			if (who & 1) {
				a = x[0];
				b = Read(0x29) - xo;
				for (i = 15; i > 0; i--) {
					x[i] = x[i - 1];
					a += (signed short) x[i];
				}
				x[0] = b;
				a = a >> 2;
				if (a >= 0) {
					PWM[0] = a;
					PWM[2] = 0;
				} else {
					PWM[0] = 0;
					PWM[2] = -a;
				}
			}

			if (who & 2) {
				a = y[0];
				b = Read(0x2B) - yo;
				for (i = 15; i > 0; i--) {
					y[i] = y[i - 1];
					a += (signed short) y[i];
				}
				y[0] = b;
				a = a >> 2;
				if (a >= 0) {
					PWM[1] = a;
					PWM[3] = 0;
				} else {
					PWM[1] = 0;
					PWM[3] = -a;
				}
			}
		}
	}

	TIM7->DIER = 0x0000; // Update Int disable
	while (1) {
		for (i = 0; i < 0x1000000; i++) { //bekle
		}
		//GPIOD->ODR ^= 0x0000F000; //tum ledleri yak //bu satira gerek yok, o pinleri baska bir is icin kullanamayiz yoksa
	}

}
