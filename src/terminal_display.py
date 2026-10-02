# =========================
# DISPLAY (TWO-COLUMN, BOXED, YELLOW VALUES, COLOUR-SAFE)
# =========================

# ANSI COLOURS
CYAN   = "\033[96m"
YELLOW = "\033[93m"
GREEN  = "\033[92m"
RED    = "\033[91m"
RESET  = "\033[0m"

def colour_bool(value):
    """Returns ON/OFF with colour coding."""
    return f"{GREEN}ON{RESET}" if value else f"{RED}OFF{RESET}"

def colour_yesno(value):
    """Returns YES/NO with colour coding."""
    return f"{GREEN}YES{RESET}" if value else f"{RED}NO{RESET}"

def colour_status(text, safe=True):
    """General safe/unsafe colour wrapper."""
    return f"{GREEN}{text}{RESET}" if safe else f"{RED}{text}{RESET}"

def box(title, lines):
    """Builds a Unicode bordered panel with colour-coded contents."""
    width = 29
    top    = f"┌{'─'*width}┐"
    header = f"│ {CYAN}{title.center(width-2)}{RESET} │"
    sep    = f"├{'─'*width}┤"
    bottom = f"└{'─'*width}┘"

    body = []
    for label, val in lines:
        body.append(f"│ {CYAN}{label:<12}{RESET} {val:<14}│")

    return [top, header, sep] + body + [bottom]

def print_columns(left, right):
    """Prints two equal-height panels side by side."""
    for l, r in zip(left, right):
        print(f"{l}   {r}")

# Clear screen
print("\033[2J\033[H", end="")


# =========================
# BUILD EACH PANEL
# =========================

# ATTACK DRONE PANEL
attack_panel = box("ATTACK DRONE", [
    ("Mode:",     f"{YELLOW}{attack.mode}{RESET}"),
    ("Lat:",      f"{YELLOW}{attack.lat:.6f}{RESET}"),
    ("Lon:",      f"{YELLOW}{attack.lon:.6f}{RESET}"),
    ("Alt(rel):", colour_status(f"{attack.alt_rel:.1f} m", attack.alt_rel >= 10)),
    ("Alt(MSL):", f"{YELLOW}{attack.alt_msl:.1f} m{RESET}"),
    ("Vn/Ve:",    f"{YELLOW}{attack.vx:.1f}/{attack.vy:.1f}{RESET}"),
])

# TARGET DRONE PANEL
target_panel = box("TARGET DRONE", [
    ("Lat:",      f"{YELLOW}{target.lat:.6f}{RESET}"),
    ("Lon:",      f"{YELLOW}{target.lon:.6f}{RESET}"),
    ("Alt(rel):", f"{YELLOW}{target.alt_rel:.1f} m{RESET}"),
    ("Alt(MSL):", f"{YELLOW}{target.alt_msl:.1f} m{RESET}"),
    ("Speed:",    f"{YELLOW}{target_speed:.1f} m/s{RESET}"),
])

# RELATIVE PANEL
relative_panel = box("RELATIVE", [
    ("Dist:",      f"{YELLOW}{distance:.1f} m{RESET}"),
    ("Bearing:",   f"{YELLOW}{bearing:.1f}°{RESET}"),
    ("TTC:",       f"{YELLOW}{time_to_intercept:.1f} s{RESET}"
                    if time_to_intercept else f"{RED}---{RESET}"),
])


# GUIDANCE PANEL
display_mode_text = (
    f"{RED}RTL{RESET}" if rtb_latched else
    f"{RED}ATTACK{RESET}" if mode == "ATTACK" else
    f"{GREEN}{mode}{RESET}"
)

guidance_panel = box("GUIDANCE", [
    ("Mode:",     display_mode_text),
    ("CmdSpeed:", f"{YELLOW}{commanded_speed:.1f} m/s{RESET}"),
    ("Heading:",  f"{YELLOW}{desired_heading:.1f}°{RESET}"),
])


# FAILSAFES PANEL
failsafe_panel = box("FAILSAFES", [
    ("ChuteArm:", colour_yesno(rc_parachute)),
    ("ChuteDep:", colour_yesno(parachute_deployed)),
    ("RTB:",      colour_bool(rc_rtb)),
])


# RC INPUT PANEL
rc_panel = box("RC INPUTS", [
    ("ATTACK:",    colour_bool(rc_attack)),
    ("ABORT:",     colour_bool(rc_abort)),
    ("RTB:",       colour_bool(rc_rtb)),
    ("PARA:",      colour_bool(rc_parachute)),
])


# HOME PANEL (single column printed after main layout)
if distance_to_home is not None:
    home_text = colour_status(f"{distance_to_home:.1f} m", distance_to_home < 1500)
else:
    home_text = f"{RED}---{RESET}"

home_panel = box("HOME", [
    ("Dist Home:", home_text),
])


# =========================
# PRINT FINAL TWO-COLUMN LAYOUT
# =========================

print_columns(attack_panel, target_panel)
print()
print_columns(relative_panel, guidance_panel)
print()
print_columns(failsafe_panel, rc_panel)
print()
for line in home_panel:
    print(line)

print()
time.sleep(0.05)
