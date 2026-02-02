package frc.entech.NavX;

interface IBoardCapabilities {
    public boolean isOmniMountSupported();

    public boolean isBoardYawResetSupported();

    public boolean isDisplacementSupported();

    public boolean isAHRSPosTimestampSupported();
}
