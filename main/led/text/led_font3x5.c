#include "led_font3x5.h"

#include <stddef.h>

// Glyph representations of all 26 upper-case letters

static const led_font_3x5_glyph_t s_letter_a = {
   .glyph = {
      {0, 1, 0},
      {1, 0, 1},
      {1, 1, 1},
      {1, 0, 1},
      {1, 0, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_b = {
   .glyph = {
      {1, 1, 0},
      {1, 0, 1},
      {1, 1, 0},
      {1, 0, 1},
      {1, 1, 0}
   }
};

static const led_font_3x5_glyph_t s_letter_c = {
   .glyph = {
      {0, 1, 1},
      {1, 0, 0},
      {1, 0, 0},
      {1, 0, 0},
      {0, 1, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_d = {
   .glyph = {
      {1, 1, 0},
      {1, 0, 1},
      {1, 0, 1},
      {1, 0, 1},
      {1, 1, 0}
   }
};

static const led_font_3x5_glyph_t s_letter_e = {
   .glyph = {
      {1, 1, 1},
      {1, 0, 0},
      {1, 1, 0},
      {1, 0, 0},
      {1, 1, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_f = {
   .glyph = {
      {1, 1, 1},
      {1, 0, 0},
      {1, 1, 0},
      {1, 0, 0},
      {1, 0, 0}
   }
};

static const led_font_3x5_glyph_t s_letter_g = {
   .glyph = {
      {0, 1, 1},
      {1, 0, 0},
      {1, 0, 1},
      {1, 0, 1},
      {0, 1, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_h = {
   .glyph = {
      {1, 0, 1},
      {1, 0, 1},
      {1, 1, 1},
      {1, 0, 1},
      {1, 0, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_i = {
   .glyph = {
      {1, 1, 1},
      {0, 1, 0},
      {0, 1, 0},
      {0, 1, 0},
      {1, 1, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_j = {
   .glyph = {
      {0, 0, 1},
      {0, 0, 1},
      {0, 0, 1},
      {1, 0, 1},
      {0, 1, 0}
   }
};

static const led_font_3x5_glyph_t s_letter_k = {
   .glyph = {
      {1, 0, 1},
      {1, 0, 1},
      {1, 1, 0},
      {1, 0, 1},
      {1, 0, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_l = {
   .glyph = {
      {1, 0, 0},
      {1, 0, 0},
      {1, 0, 0},
      {1, 0, 0},
      {1, 1, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_m = {
   .glyph = {
      {1, 0, 1},
      {1, 1, 1},
      {1, 1, 1},
      {1, 0, 1},
      {1, 0, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_n = {
   .glyph = {
      {1, 0, 1},
      {1, 1, 1},
      {1, 1, 1},
      {1, 1, 1},
      {1, 0, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_o = {
   .glyph = {
      {0, 1, 0},
      {1, 0, 1},
      {1, 0, 1},
      {1, 0, 1},
      {0, 1, 0}
   }
};

static const led_font_3x5_glyph_t s_letter_p = {
   .glyph = {
      {1, 1, 0},
      {1, 0, 1},
      {1, 1, 0},
      {1, 0, 0},
      {1, 0, 0}
   }
};

static const led_font_3x5_glyph_t s_letter_q = {
   .glyph = {
      {0, 1, 0},
      {1, 0, 1},
      {1, 0, 1},
      {1, 1, 1},
      {0, 1, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_r = {
   .glyph = {
      {1, 1, 0},
      {1, 0, 1},
      {1, 1, 0},
      {1, 0, 1},
      {1, 0, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_s = {
   .glyph = {
      {0, 1, 1},
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1},
      {1, 1, 0}
   }
};

static const led_font_3x5_glyph_t s_letter_t = {
   .glyph = {
      {1, 1, 1},
      {0, 1, 0},
      {0, 1, 0},
      {0, 1, 0},
      {0, 1, 0}
   }
};

static const led_font_3x5_glyph_t s_letter_u = {
   .glyph = {
      {1, 0, 1},
      {1, 0, 1},
      {1, 0, 1},
      {1, 0, 1},
      {1, 1, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_v = {
   .glyph = {
      {1, 0, 1},
      {1, 0, 1},
      {1, 0, 1},
      {1, 0, 1},
      {0, 1, 0}
   }
};

static const led_font_3x5_glyph_t s_letter_w = {
   .glyph = {
      {1, 0, 1},
      {1, 0, 1},
      {1, 1, 1},
      {1, 1, 1},
      {1, 0, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_x = {
   .glyph = {
      {1, 0, 1},
      {1, 0, 1},
      {0, 1, 0},
      {1, 0, 1},
      {1, 0, 1}
   }
};

static const led_font_3x5_glyph_t s_letter_y = {
   .glyph = {
      {1, 0, 1},
      {1, 0, 1},
      {0, 1, 0},
      {0, 1, 0},
      {0, 1, 0}
   }
};

static const led_font_3x5_glyph_t s_letter_z = {
   .glyph = {
      {1, 1, 1},
      {0, 0, 1},
      {0, 1, 0},
      {1, 0, 0},
      {1, 1, 1}
   }
};

// Blank glyph for spaces between words
static const led_font_3x5_glyph_t s_space = {
   .glyph = {
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0}
   }
};

// Question mark
static const led_font_3x5_glyph_t s_question = {
   .glyph = {
      {1, 1, 0},
      {0, 0, 1},
      {0, 1, 0},
      {0, 0, 0},
      {0, 1, 0}
   }
};

// Exclamation mark
static const led_font_3x5_glyph_t s_exclamation = {
   .glyph = {
      {0, 1, 0},
      {0, 1, 0},
      {0, 1, 0},
      {0, 0, 0},
      {0, 1, 0}
   }
};

// Return the 3x5 glyph for supported character
const led_font_3x5_glyph_t *led_font_3x5_get_glyph(char character)
{
   // Normalize lowercase letters to uppercase
   if (character >= 'a' && character <= 'z')
      character -= ('a' - 'A');

   switch (character)
   {
      case 'A': return &s_letter_a;
      case 'B': return &s_letter_b;
      case 'C': return &s_letter_c;
      case 'D': return &s_letter_d;
      case 'E': return &s_letter_e;
      case 'F': return &s_letter_f;
      case 'G': return &s_letter_g;
      case 'H': return &s_letter_h;
      case 'I': return &s_letter_i;
      case 'J': return &s_letter_j;
      case 'K': return &s_letter_k;
      case 'L': return &s_letter_l;
      case 'M': return &s_letter_m;
      case 'N': return &s_letter_n;
      case 'O': return &s_letter_o;
      case 'P': return &s_letter_p;
      case 'Q': return &s_letter_q;
      case 'R': return &s_letter_r;
      case 'S': return &s_letter_s;
      case 'T': return &s_letter_t;
      case 'U': return &s_letter_u;
      case 'V': return &s_letter_v;
      case 'W': return &s_letter_w;
      case 'X': return &s_letter_x;
      case 'Y': return &s_letter_y;
      case 'Z': return &s_letter_z;
      case ' ': return &s_space;
      case '?': return &s_question;
      case '!': return &s_exclamation;
      default:  return NULL;
   }
}