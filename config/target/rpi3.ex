use Mix.Config

# We need a special fwup conf for the initial v6 release.
# TODO Remove this some day.
config :nerves, :firmware, fwup_conf: "fwup_interim.conf"

config :bootloader,
  init: [:nerves_runtime, :nerves_firmware_ssh],
  app: :farmbot
