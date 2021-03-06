package org.usfirst.frc.team2077.commands;
import org.junit.Test;
import org.usfirst.frc.team2077.commands.Move2;

import static org.junit.Assert.*;
import static org.junit.Assert.assertEquals;

public class Move2Test {
    @Test
    public void testThings() throws Exception {
        assertEquals(Move2.bestRotation(180), -180, 0.1);
        assertEquals(Move2.bestRotation(190), -170, 0.1);
        assertEquals(Move2.bestRotation(370), 10, 0.1);
        assertEquals(Move2.bestRotation(220), -140, 0.1);        
        
        assertEquals(Move2.bestRotation(1080), 0, 0.1);
        assertEquals(Move2.bestRotation(360), 0, 0.1);
        assertEquals(Move2.bestRotation(720), 0, 0.1);
    }
}