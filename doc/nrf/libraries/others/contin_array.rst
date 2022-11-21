.. _lib_contin_array:

Continuous array
################

.. contents::
   :local:
   :depth: 2

The continuous array library introduces an array that you can loop over, for example if you want to create a period of the Pulse-code modulated (PCM) sine wave.
You can use it to test playback with applications that support audio development kits, for example :ref:`nrf53_audio_app`.

Overview
********

The library introduces the :c:func:`contin_array_create` function, which takes an array that the user wants to loop over.
The contents of the target array are supplied to ``pcm_finite`` and information about its size to ``pcm_finite_size``.
The user may then call the function to populate the destination array with the pointer ``pcm_cont`` with ``pcm_cont_size`` bytes of data.
This allows creating ``N`` repetitions of ``pcm_finite`` into ``pcm_cont``, with data fetched in smaller pieces into RAM.

The function keeps track of the current position in ``pcm_finite``, so that the function can be called multiple times and maintain the correct position in ``pcm_finite``.
Wrapping is handled automatically.

Configuration
*************

To enable the library, set the :kconfig:option:`CONFIG_CONTIN_ARRAY` Kconfig option to ``y`` in the project configuration file :file:`prj.conf`.

Usage
*****

The following |NCS| applications use this library:

* :ref:`nrf53_audio_app`

API documentation
*****************

| Header file: :file:`include/contin_array.h`
| Source file: :file:`lib/contin_array/contin_array.c`

.. doxygengroup:: contin_array
   :project: nrf
   :members:
