local MAV_SEVERITY_INFO = 6
CHAN = 9 -- AUX1 for NEOPIXEL
C_CHAN = 13 -- AUX6 for COLLISION LIGHT


local FLASH_HZ = 3

local last_flash = millis()
local led_on = false
local cled_on = false
local rc7_on = false
local armed = true

local function set_navigation(state)
    if (state == true) then
        serialLED:set_RGB(CHAN, 0, 255, 0, 0)
        serialLED:set_RGB(CHAN, 1, 255, 0, 0)
        serialLED:set_RGB(CHAN, 2, 0, 255, 0)
        serialLED:set_RGB(CHAN, 3, 0, 255, 0)  
        serialLED:send(CHAN)
        led_on = true
    else
        serialLED:set_RGB(CHAN, -1, 0, 0, 0)
        serialLED:send(CHAN)
        led_on = false
    end
end

local function set_collision(state)
    if (state == true) then
        SRV_Channels:set_output_pwm_chan(C_CHAN, 2000)
        cled_on = true
    else 
        SRV_Channels:set_output_pwm_chan(C_CHAN, 1000)
        cled_on = false
    end
end

local function update_collision()
    if (rc:get_pwm(7) > 1800) and (rc7_on == false) then
        rc7_on = true
        if (cled_on == true) then
            set_collision(false)
        else
            set_collision(true)
        end
    elseif (rc:get_pwm(7) < 1200) and (rc7_on == true) then
        rc7_on = false
    end
end

function update_LEDs()
    update_collision()
    if ((arming:is_armed() == true) and (armed == false)) then
        armed = true
        set_collision(true)
        set_navigation(true)
    elseif ((arming:is_armed() == false) and (armed == true)) then
        armed = false
        set_collision(false)

    elseif (armed == false) and ((millis() - last_flash) > (1000 / FLASH_HZ)) then
        last_flash = millis()
        if (led_on == true) then
            set_navigation(false)
        else
            set_navigation(true)
        end 
    end

    return update_LEDs, 100
end

serialLED:set_num_neopixel(CHAN, 4)
set_navigation(true)

gcs:send_text(MAV_SEVERITY_INFO, 'light script active')
return update_LEDs, 1000
