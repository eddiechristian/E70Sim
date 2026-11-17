/* With its "device" feature on, cortex-m-rt's link.x includes this file.
   A chip support crate would use it to give every interrupt handler a
   default. Our interrupt table in src/main.rs names real functions only,
   so there is nothing to add. */
