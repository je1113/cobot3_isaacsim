import omni.ui as ui
from omni.ui import scene as sc
from omni.kit.viewport.utility import get_active_viewport_window


_viewport_window = None
_zone_scene_view = None


def create_zone_labels():
    global _viewport_window
    global _zone_scene_view

    _viewport_window = get_active_viewport_window()

    if _viewport_window is None:
        print("[ZoneLabels] No active viewport found.")
        return

    # Prevent duplicate labels if the script is run again
    if _zone_scene_view is not None:
        try:
            _viewport_window.viewport_api.remove_scene_view(_zone_scene_view)
        except Exception:
            pass

    with _viewport_window.get_frame("factory_zone_labels"):

        _zone_scene_view = sc.SceneView()

        _viewport_window.viewport_api.add_scene_view(
            _zone_scene_view
        )

        with _zone_scene_view.scene:

            def add_label(text, x, y, z):

                # World position
                with sc.Transform(
                    transform=sc.Matrix44.get_translation_matrix(
                        x, y, z
                    )
                ):

                    # Always face camera and keep readable screen size
                    with sc.Transform(
                        look_at=sc.Transform.LookAt.CAMERA,
                        scale_to=sc.Space.SCREEN
                    ):

                        sc.Label(
                            text,
                            size=28,
                            color=0xFFFFFFFF,
                            alignment=ui.Alignment.CENTER
                        )

            # Left
            add_label(
                "Packaging Zone",
                6.4,
                4.6,
                2.5
            )

            # Middle
            add_label(
                "Unloader Zone",
                6.4,
                0.9713649,
                2.5
            )

            # Right
            add_label(
                "Testing Zone",
                6.4,
                -2.7053146,
                2.5
            )

    print("[ZoneLabels] Labels created.")


def remove_zone_labels():
    global _viewport_window
    global _zone_scene_view

    if _viewport_window is not None and _zone_scene_view is not None:
        try:
            _viewport_window.viewport_api.remove_scene_view(
                _zone_scene_view
            )
        except Exception:
            pass

    _zone_scene_view = None

    print("[ZoneLabels] Labels removed.")


# Run automatically when script is loaded
create_zone_labels()