/* Remap NV12 planes on Intel GPU surfaces with the full calibrated lens model. */
// Camera intrinsics, target projection, and chroma offsets are baked once at startup.
// CALIBRATION_CONSTANTS

// OpenCL texel centers are integer + 0.5; preserve fractional map coordinates.
const sampler_t linear_sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_LINEAR;

/* Map destination luma coordinates to the distorted source image. */
float2 source_position(float2 point)
{
    float x = (point.x - NCX) / NFX;
    float y = (point.y - NCY) / NFY;
    float radius = x*x + y*y;
    float radial = 1.0f + radius*(K1 + radius*(K2 + radius*K3));
    return (float2)(FX*(x*radial + 2.0f*P1*x*y + P2*(radius + 2.0f*x*x)) + CX,
                    FY*(y*radial + P1*(radius + 2.0f*y*y) + 2.0f*P2*x*y) + CY);
}

/* Keep the same geometry on Y and correctly sited interleaved UV. */
__kernel void rectify(__write_only image2d_t dst, uint index, __read_only image2d_t src)
{
    int2 pixel = (int2)(get_global_id(0), get_global_id(1));
    int chroma = get_image_width(src) == (int)IMAGE_WIDTH / 2;
    float scale = chroma ? 2.0f : 1.0f;
    float2 offset = chroma ? (float2)(CHROMA_X, 0.5f) : (float2)(0.0f);
    float2 point = convert_float2(pixel)*scale + offset;
    float2 mapped = (source_position(point) - offset)/scale;
    // VAAPI images may include coded-height padding below the visible image.
    mapped = clamp(mapped, (float2)(0.0f), (float2)(IMAGE_WIDTH/scale - 1.0f, IMAGE_HEIGHT/scale - 1.0f));
    write_imagef(dst, pixel, read_imagef(src, linear_sampler, mapped + 0.5f));
}
