<h1>ILI9341 Driver for MPLAB harmony V1.11</h1>


<h3>Description</h3>
<p>I wrote this driver to use ILI9341 with MPLAB harmony graphics library. Currently it is supporting only 16bit mode via PORTB, and SPI mode with static SPI driver.<p>


<h3>Features</h3>
<ul>
	<li>Configurable through MPLAB Harmony Configurator</li>
	<li>16bit paralel mode</li>
	<li>4pin SPI mode (tested up to 50Mhz data rate</li>
</ul>


<h3>How to use</h3>
<ul>
	<li>Download and copy it to harmony installation path(overwrite files)</li>
	<li>Set appropriate pins to Digital Output</li>
	<li>In the case of SPI configure it to static mode</li>
	<li>Set up graphics library</li>
	<li>Compile and enjoy</li>
</ul>

<h3>Future</h3>
I will implement 8bit mode, and use PMP module instead of PORTB