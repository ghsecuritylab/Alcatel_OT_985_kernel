class Surface
{
public:
    Surface(/*what?*/);

    void setFont(RenderingFontInstance *font);

    void drawGlyphs(RenderingFontInstance *font, const LEGlyphID *glyphs, le_int32 count, const le_int32 *dx,
        le_int32 x, le_int32 y, le_int32 width, le_int32 height);
};