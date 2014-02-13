const int oneGrid = 562 * 10 / 20;
const int one360 = 1650;

void goAhead(float grid) {
    int need = grid * oneGrid;

    if (need < 0) {
        need *= -1;
        md.setSpeeds(-200, -200);
    } else {
        md.setSpeeds(200, 200);
    }

    while (need) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
        need--;
    }

    md.setBrakes(400, 400);
}

void rotateLeft(int degree) { // require md, encoder left, encoder righ, 
    int need = degree / 360 * one360;

    if (degree < 0) {
        need *= -1;
        md.setSpeeds(200, -200);
    } else {
        md.setSpeeds(200, -200);
    }

    while (need) {
        while (digitalRead(enLeft));
        while (!digitalRead(enLeft));
        need--;
    }

    md.setBrakes(400, 400);
}

void shiftLeft(float grid) {
    rotateLeft(45);
    goAhead(1.414 * grid);
    rotateLeft(-45);
    goAhead(-1.414 * grid);
}

void selfAdjust(float leftHead, float leftTail) {
    if (14 < leftHead && 14 < leftTail) {

    } else if (10 < leftHead && leftHead < 14 && 10 < leftTail && leftTail < 14) {
        shiftLeft(1.5);
    } else if (7 < leftHead && leftHead < 10 && 7 < leftTail && leftTail < 10) {
        shiftLeft(1);
    } else {
        shiftLeft(-1);
    }
}

// speed(encoder feedback)
// keepStraight(encoder feedback)

// sense twice and wait enough time for stable data

// raise the direction of ultrasonic if placed vertical
