#include "debug.h"
#include "../uvos/uvos_usart.h"
#include "../uvos/hardware.h"


void debug_init( void )
{
	debug_putc( '\n' );
	debug( "###  UAVWare  ###\n" );
	debug( "debug: init done\n" );
	debug_cls();
}

void debug_cls( void )
{
	// debug_putc(0x1B); // \e, Escape
	// debug("\033[2J\r");
	debug( "\x1b[2J" );
}

void debug_putc( uint8_t c )
{
	// add \r to newlines
	// if (ch == '\n') debug_putc('\r');
	UVOS_USART_debug_putc( c );
}

void debug_puts( char * data, uint16_t len )
{
	uint8_t c = *data++;
	while ( len > 0U ) {
		debug_putc( c );
		c = *data++;
		len--;
	}
}

void debug(const char * data)
{
	uint8_t c = *data++;
	while ( c ) {
		debug_putc( c );
		c = *data++;
	}
}

// put hexadecimal number to debug out.
void debug_put_hex8( uint8_t val )
{
	uint8_t lo = val & 0x0F;
	uint8_t hi = val >> 4;
	if ( hi < 0x0A ) {
		hi = '0' + hi;
	} else {
		hi = 'A' - 0x0A + hi;
	}

	if ( lo < 0x0A ) {
		lo = '0' + lo;
	} else {
		lo = 'A' - 0x0A + lo;
	}
	debug_putc( hi );
	debug_putc( lo );
}

// put 16bit hexadecimal number to debug out
void debug_put_hex16( uint16_t val )
{
	debug_put_hex8( val >> 8 );
	debug_put_hex8( val & 0xFF );
}

// put 32bit hexadecimal number to debug out
void debug_put_hex32( uint32_t val )
{
	debug_put_hex8( val >> 24 );
	debug_put_hex8( val >> 16 );
	debug_put_hex8( val >> 8 );
	debug_put_hex8( val & 0xFF );
}

// output a signed 8-bit number to uart
void debug_put_int8( int8_t c )
{
	uint8_t tmp;
	uint8_t mul;
	uint8_t l;
	uint8_t uint_s;

	if ( c < 0 ) {
		debug_putc( '-' );
		uint_s = -c;
	} else {
		uint_s = c;
	}

	l = 0;
	for ( mul = 100; mul > 0; mul = mul / 10 ) {
		tmp = '0';
		while ( uint_s >= mul ) {
			uint_s -= mul;
			tmp++;
			l = 1;
		}
		if ( ( l == 0 ) && ( tmp == '0' ) && ( mul != 1 ) ) {
			// dont print spacer
			// debug_putc(' ');
		} else {
			debug_putc( tmp );
		}
	}
}

// output an unsigned 8-bit number to uart
void debug_put_uint8( uint8_t c )
{
	uint8_t tmp;
	uint8_t mul;
	uint8_t l;

	l = 0;
	for ( mul = 100; mul > 0 ; mul = mul / 10 ) {
		tmp = '0';
		while ( c >= mul ) {
			c -= mul;
			tmp++;
			l = 1;
		}
		if ( ( l == 0 ) && ( tmp == '0' ) && ( mul != 1 ) ) {
			// dont print spacer
			// debug_putc(' ');
		} else {
			debug_putc( tmp );
		}
	}
}

// output an unsigned 16-bit number to uart
void debug_put_uint16( uint16_t c )
{
	uint8_t tmp;
	uint8_t l = 0;
	// loop unrolling is better(no int16 arithmetic)
	/*for (mul = 10000; mul>0; mul = mul/ 10) {
	    uint16_t mul;

	    l = 0;
	            tmp = '0';
	            while (c>=mul) {
	                    c -= mul;
	                    tmp++;
	                    l = 1;
	            }
	            if ((l == 0) && (tmp == '0') && (mul!=1)) {
	                    // debug_putc(' ');
	            } else {
	                    debug_putc(tmp);
	            }
	    }*/
	tmp = 0;
	while ( c >= 10000L ) {
		c -= 10000L;
		tmp++;
		l = 1;
	}
	if ( tmp != 0 ) debug_putc( '0' + tmp );

	tmp = 0;
	while ( c >= 1000L ) {
		c -= 1000L;
		tmp++;
		l = 1;
	}
	if ( l || ( tmp != 0 ) ) debug_putc( '0' + tmp );

	tmp = 0;
	while ( c >= 100 ) {
		c -= 100;
		tmp++;
		l = 1;
	}
	if ( l || ( tmp != 0 ) ) debug_putc( '0' + tmp );

	tmp = 0;
	while ( c >= 10 ) {
		c -= 10;
		tmp++;
		l = 1;
	}
	if ( l || ( tmp != 0 ) ) debug_putc( '0' + tmp );

	debug_putc( '0' + ( uint8_t )c );
}

void debug_put_fixed2( uint16_t c )
{
	uint8_t tmp;
	uint8_t l = 0;
	tmp = 0;
	while ( c >= 10000L ) {
		c -= 10000L;
		tmp++;
		l = 1;
	}
	if ( tmp != 0 ) debug_putc( '0' + tmp );

	tmp = 0;
	while ( c >= 1000L ) {
		c -= 1000L;
		tmp++;
		l = 1;
	}
	if ( l || ( tmp != 0 ) ) debug_putc( '0' + tmp );

	tmp = 0;
	while ( c >= 100 ) {
		c -= 100;
		tmp++;
		l = 1;
	}
	if ( l || ( tmp != 0 ) ) debug_putc( '0' + tmp );

	debug_putc( '.' );

	tmp = 0;
	while ( c >= 10 ) {
		c -= 10;
		tmp++;
		l = 1;
	}
	if ( l || ( tmp != 0 ) ) debug_putc( '0' + tmp );

	debug_putc( '0' + ( uint8_t )c );
}
