const COLORS = {
    ground:   [255, 255, 255, 255],
    obstacle: [  0,   0,   0, 255],
    unknown:  [  0,   0,   0,   0],
    clean:    [100, 200, 255, 150],
    keepout:  [220,  20,  60, 200]
}

export function gridBytesToImageData(data, width, height, overlayType = null) {
    const arr = new Uint8ClampedArray(width * height * 4);
    
    let i = 0;

    for (let y = 0; y < height; y++) {
        // Read the ROS grid from bottom to top
        const srcRow = (height - 1 - y) * width;

        for (let x = 0; x < width; x++) {
            const val = data[srcRow + x];
            
            if (overlayType === 'vacuum') {
                if (val === 100)      arr.set(COLORS.clean, i);
                else if (val === 0)   arr.set(COLORS.unknown, i);
            } else if (overlayType === 'keepout') {
                if (val === 100)      arr.set(COLORS.keepout, i);
                else if (val === 0)   arr.set(COLORS.unknown, i);
            } else {
                if (val === 0)        arr.set(COLORS.ground,   i);
                else if (val === 100) arr.set(COLORS.obstacle, i);
                else                  arr.set(COLORS.unknown,  i);
            }
            
            i += 4;
        }
    }
    
    return new ImageData(arr, width, height);
}
