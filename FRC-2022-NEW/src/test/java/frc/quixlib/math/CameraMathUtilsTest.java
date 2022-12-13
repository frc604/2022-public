package frc.quixlib.math;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import org.junit.jupiter.api.*;

public class CameraMathUtilsTest {
  static final double kTol = 1e-6;

  @Test
  public void cart2BEPinhole2CartTest() {
    var vec = Matrix.mat(Nat.N3(), Nat.N1()).fill(1.0, -1.0, 1.0);
    var pinhole = CameraMathUtils.cart2BEPinhole(vec);
    assertTrue(pinhole.getFirst() > 0.0); // Check +bearing to the right.
    assertTrue(pinhole.getSecond() > 0.0); // Check +elevation is up.
    var cart = CameraMathUtils.pinholeBE2Cart(pinhole.getFirst(), pinhole.getSecond());
    assertTrue(vec.div(vec.normF()).isEqual(cart.div(cart.normF()), kTol));

    vec = Matrix.mat(Nat.N3(), Nat.N1()).fill(1.23, 4.56, -7.89);
    pinhole = CameraMathUtils.cart2BEPinhole(vec);
    assertTrue(pinhole.getFirst() < 0.0); // Check -bearing to the left.
    assertTrue(pinhole.getSecond() < 0.0); // Check -elevation is down.
    cart = CameraMathUtils.pinholeBE2Cart(pinhole.getFirst(), pinhole.getSecond());
    assertTrue(vec.div(vec.normF()).isEqual(cart.div(cart.normF()), kTol));
  }

  @Test
  public void cart2BESph2CartTest() {
    var vec = Matrix.mat(Nat.N3(), Nat.N1()).fill(1.0, -1.0, 1.0);
    var pinhole = CameraMathUtils.cart2BESph(vec);
    assertTrue(pinhole.getFirst() < 0.0); // Check -bearing to the left.
    assertTrue(pinhole.getSecond() > 0.0); // Check +elevation is up.
    var cart = CameraMathUtils.beSph2Cart(pinhole.getFirst(), pinhole.getSecond());
    assertTrue(vec.div(vec.normF()).isEqual(cart.div(cart.normF()), kTol));

    vec = Matrix.mat(Nat.N3(), Nat.N1()).fill(1.23, 4.56, -7.89);
    pinhole = CameraMathUtils.cart2BESph(vec);
    assertTrue(pinhole.getFirst() > 0.0); // Check +bearing to the right.
    assertTrue(pinhole.getSecond() < 0.0); // Check -elevation is down.
    cart = CameraMathUtils.beSph2Cart(pinhole.getFirst(), pinhole.getSecond());
    assertTrue(vec.div(vec.normF()).isEqual(cart.div(cart.normF()), kTol));
  }
}
